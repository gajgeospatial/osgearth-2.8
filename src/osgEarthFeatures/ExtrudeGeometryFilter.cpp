/* -*-c++-*- */
/* osgEarth - Dynamic map generation toolkit for OpenSceneGraph
 * Copyright 2016 Pelican Mapping
 * http://osgearth.org
 *
 * osgEarth is free software; you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#include <osgEarthFeatures/ExtrudeGeometryFilter>
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthSymbology/ResourceCache>
#include <osgEarth/ECEF>
#include <osgEarth/ImageUtils>
#include <osgEarth/Clamping>
#include <osgEarth/Utils>
#include <osgEarth/Tessellator>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osgUtil/Tessellator>
#include <osgUtil/Optimizer>
#include <osgUtil/SmoothingVisitor>
#include <osgUtil/Simplifier>
#include <osg/LineWidth>
#include <osg/PolygonOffset>
#include <osg/FrontFace>
#include <osg/CullFace>
#include <osg/ValueObject>

#include <gdal_priv.h>
#include <gdalwarper.h>
#include <ogr_core.h>
#include <ogrsf_frmts.h>

#define LC "[ExtrudeGeometryFilter] "

#define GEOTRSFRM_TOPLEFT_X            0
#define GEOTRSFRM_WE_RES               1
#define GEOTRSFRM_ROTATION_PARAM1      2
#define GEOTRSFRM_TOPLEFT_Y            3
#define GEOTRSFRM_ROTATION_PARAM2      4
#define GEOTRSFRM_NS_RES               5

#define VERTICALZMAX 0.00005
using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

namespace
{
    // Calculates the rotation angle of a shape. This conanically applies to
    // buildings; it finds the longest edge and compares its angle to the
    // x-axis to determine a rotation value. This method is used so we can 
    // properly rotate textures for rooftop application.
    float getApparentRotation( const Geometry* geom )
    {
        Segment n;
        double  maxLen2 = 0.0;
        ConstSegmentIterator i( geom, true );
        while( i.hasMore() )
        {
            Segment s = i.next();
            double len2 = (s.second - s.first).length2();
            if ( len2 > maxLen2 ) 
            {
                maxLen2 = len2;
                n = s;
            }
        }

        const osg::Vec3d& p1 = n.first.x() < n.second.x() ? n.first : n.second;
        const osg::Vec3d& p2 = n.first.x() < n.second.x() ? n.second : n.first;

        return atan2( p2.x()-p1.x(), p2.y()-p1.y() );
    }
}

#define AS_VEC4(V3, X) osg::Vec4f( (V3).x(), (V3).y(), (V3).z(), X )

//------------------------------------------------------------------------

ExtrudeGeometryFilter::ExtrudeGeometryFilter() :
_mergeGeometry         ( true ),
_wallAngleThresh_deg   ( 60.0 ),
_styleDirty            ( true ),
_makeStencilVolume     ( false ),
_gpuClamping           ( false )
{
    _cosWallAngleThresh = cos( _wallAngleThresh_deg );
}

void
ExtrudeGeometryFilter::setStyle( const Style& style )
{
    _style      = style;
    _styleDirty = true;
}

void
ExtrudeGeometryFilter::reset( const FilterContext& context )
{
    _cosWallAngleThresh = cos( _wallAngleThresh_deg );
    _geodes.clear();
    
    if ( _styleDirty )
    {
        const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

        _wallSkinSymbol    = 0L;
        _wallPolygonSymbol = 0L;
        _roofSkinSymbol    = 0L;
        _roofPolygonSymbol = 0L;
        _extrusionSymbol   = 0L;
        _outlineSymbol     = 0L;

        _gpuClamping = false;

        _extrusionSymbol = _style.get<ExtrusionSymbol>();
        if ( _extrusionSymbol.valid() )
        {
            // make a copy of the height expression so we can use it:
            if ( _extrusionSymbol->heightExpression().isSet() )
            {
                _heightExpr = *_extrusionSymbol->heightExpression();
            }

            // If there is no height expression, and we have either absolute or terrain-relative
            // clamping, THAT means that we want to extrude DOWN from the geometry to the ground
            // (instead of from the geometry.)
            AltitudeSymbol* alt = _style.get<AltitudeSymbol>();
            if ( alt && !_extrusionSymbol->heightExpression().isSet() && !_extrusionSymbol->height().isSet() )
            {
                if (alt->clamping() == AltitudeSymbol::CLAMP_ABSOLUTE ||
                    alt->clamping() == AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN )
                {
                    _heightExpr = NumericExpression( "0-[__max_hat]" );
                }
            }

            // cache the GPU Clamping directive:
            if ( alt && alt->technique() == AltitudeSymbol::TECHNIQUE_GPU )
            {
                _gpuClamping = true;
            }
            
            // attempt to extract the wall symbols:
            if ( _extrusionSymbol->wallStyleName().isSet() && sheet != 0L )
            {
                const Style* wallStyle = sheet->getStyle( *_extrusionSymbol->wallStyleName(), false );
                if ( wallStyle )
                {
                    _wallSkinSymbol = wallStyle->get<SkinSymbol>();
                    _wallPolygonSymbol = wallStyle->get<PolygonSymbol>();
                }
            }

            // attempt to extract the rooftop symbols:
            if ( _extrusionSymbol->roofStyleName().isSet() && sheet != 0L )
            {
                const Style* roofStyle = sheet->getStyle( *_extrusionSymbol->roofStyleName(), false );
                if ( roofStyle )
                {
                    _roofSkinSymbol = roofStyle->get<SkinSymbol>();
                    _roofPolygonSymbol = roofStyle->get<PolygonSymbol>();
                }
            }

            // if there's a line symbol, use it to outline the extruded data.
            _outlineSymbol = _style.get<LineSymbol>();
        }

        // backup plan for skin symbols:
        const SkinSymbol* skin = _style.get<SkinSymbol>();
        if ( skin )
        {
            if ( !_wallSkinSymbol.valid() )
                _wallSkinSymbol = skin;
            if ( !_roofSkinSymbol.valid() )
                _roofSkinSymbol = skin;
        }

        // backup plan for poly symbols:
        const PolygonSymbol* poly = _style.get<PolygonSymbol>();
        if ( poly )
        {
            if ( !_wallPolygonSymbol.valid() )
                _wallPolygonSymbol = poly;
            if ( !_roofPolygonSymbol.valid() )
                _roofPolygonSymbol = poly;
        }

        _styleDirty = false;
    }
}

osg::Vec3d ExtrudeGeometryFilter::LocalizeFeature(FilterContext& cx, Feature *f)
{
    bool  makeECEF = false;
    const SpatialReference* srs = 0L;
    const SpatialReference* mapSRS = 0L;

    if (cx.isGeoreferenced())
    {
        srs = cx.extent()->getSRS();
        makeECEF = cx.getSession()->getMapInfo().isGeocentric();
        mapSRS = cx.getSession()->getMapInfo().getProfile()->getSRS();
    }
    Geometry * FeatureGeometry = f->getGeometry();

    osg::Vec2d c = FeatureGeometry->getBounds().center2d();
    osg::Vec3d centroid(c.x(), c.y(), 0.0);
    osg::Vec3d baseCentroid;
    transformAndLocalize(centroid, srs, baseCentroid, mapSRS, _world2local, makeECEF);

    GeometryIterator iter(f->getGeometry(), false);
    bool hasHoles = false;
    while (iter.hasMore())
    {
        Geometry * g = iter.next();
        if(g->getType() == Geometry::TYPE_POLYGON)
        {
#ifdef _DEBUG
            int numverts = g->getTotalPointCount();
#endif
            for (Geometry::iterator m = g->begin(); m != g->end(); ++m)
            {
                osg::Vec3d point = *m;

                // transform into target SRS.
                if (srs)
                {
                    transformAndLocalize(point, srs, point, mapSRS, _world2local, makeECEF);
                    *m = point;
                }
            }
        }
    }
    return baseCentroid;
}

bool ExtrudeGeometryFilter::SetLocalizationForFeature(FilterContext& cx, Feature* f, GeoStructure &structure)
{

    if (cx.isGeoreferenced())
    {
        structure.srs = (SpatialReference *)cx.extent()->getSRS();
        structure.makeECEF = cx.getSession()->getMapInfo().isGeocentric();
        structure.mapSRS = (SpatialReference *)cx.getSession()->getMapInfo().getProfile()->getSRS();
    }
    Geometry* FeatureGeometry = f->getGeometry();

    osg::Vec2d c = FeatureGeometry->getBounds().center2d();
    osg::Vec3d centroid(c.x(), c.y(), 0.0);

    transformAndLocalize(centroid, structure.srs, structure.baseCentroid, structure.mapSRS, _world2local, structure.makeECEF);
    return true;
}

bool
ExtrudeGeometryFilter::buildStructure(const Geometry*         input,
                                      double                  height,
                                      bool                    flatten,
                                      float                   verticalOffset,
                                      const SkinResource*     wallSkin,
                                      const SkinResource*     roofSkin,
                                      Structure&              structure,
                                      FilterContext&          cx,
									  bool					  have_roof_image,
									  std::string			  roof_image_name,
									  bool					  roof_image_has_mat_index,
									  std::string			  roof_mat_index_name,
									  ExtrudeGeomGDALHelper * RoofHelper)
{
    bool  makeECEF                 = false;
    const SpatialReference* srs    = 0L;
    const SpatialReference* mapSRS = 0L;

    if ( cx.isGeoreferenced() )
    {
       srs      = cx.extent()->getSRS();
       makeECEF = cx.getSession()->getMapInfo().isGeocentric();
       mapSRS   = cx.getSession()->getMapInfo().getProfile()->getSRS();
    }

    // whether this is a closed polygon structure.
    structure.isPolygon = (input->getComponentType() == Geometry::TYPE_POLYGON);

    // store the vert offset for later encoding
    structure.verticalOffset = verticalOffset;

    // extrusion working variables
    double     targetLen = -DBL_MAX;
    osg::Vec3d minLoc(DBL_MAX, DBL_MAX, DBL_MAX);
    double     minLoc_len = DBL_MAX;
    osg::Vec3d maxLoc(0,0,0);
    double     maxLoc_len = 0;

    // Initial pass over the geometry does two things:
    // 1: Calculate the minimum Z across all parts.
    // 2: Establish a "target length" for extrusion
    double absHeight = fabs(height);

    ConstGeometryIterator zfinder( input );
    while( zfinder.hasMore() )
    {
        const Geometry* geom = zfinder.next();
        for( Geometry::const_iterator m = geom->begin(); m != geom->end(); ++m )
        {
            osg::Vec3d m_point = *m;

            if ( m_point.z() + absHeight > targetLen )
                targetLen = m_point.z() + absHeight;

            if (m_point.z() < minLoc.z())
                minLoc = m_point;

            if (m_point.z() > maxLoc.z())
                maxLoc = m_point;
        }
    }

    osg::Vec2d c = input->getBounds().center2d();
    osg::Vec3d centroid(c.x(), c.y(), minLoc.z());
    transformAndLocalize(centroid, srs, structure.baseCentroid, mapSRS, _world2local, makeECEF );

    // apply the height offsets
    //height    -= heightOffset;
    //targetLen -= heightOffset;
    
    float   roofRotation  = 0.0f;
    Bounds  roofBounds;
    float   sinR = 0.0f, cosR = 0.0f;
    double  roofTexSpanX = 0.0, roofTexSpanY = 0.0;
    osg::ref_ptr<const SpatialReference> roofProjSRS;

    if ( roofSkin )
    {
        roofBounds = input->getBounds();

        // if our data is lat/long, we need to reproject the geometry and the bounds into a projected
        // coordinate system in order to properly generate tex coords.
        if ( srs && srs->isGeographic() )
        {
            osg::Vec2d geogCenter = roofBounds.center2d();

            // This sometimes fails with the aerodrom stuff. No idea why -gw.
            //roofProjSRS = srs->createUTMFromLonLat( Angle(geogCenter.x()), Angle(geogCenter.y()) );
            roofProjSRS = SpatialReference::create("spherical-mercator");
            if ( roofProjSRS.valid() )
            {
                roofBounds.transform( srs, roofProjSRS.get() );
                osg::ref_ptr<Geometry> projectedInput = input->clone();
                srs->transform( projectedInput->asVector(), roofProjSRS.get() );
                roofRotation = getApparentRotation( projectedInput.get() );
            }
        }
        else
        {
            roofRotation = getApparentRotation( input );
        }
            
        sinR = sin(roofRotation);
        cosR = cos(roofRotation);

        if ( !roofSkin->isTiled().value() )
        {
            //note: non-tiled roofs don't really work atm.
            roofTexSpanX = cosR*roofBounds.width() - sinR*roofBounds.height();
            roofTexSpanY = sinR*roofBounds.width() + cosR*roofBounds.height();
        }
        else
        {
            roofTexSpanX = roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : 10.0;
            if ( roofTexSpanX <= 0.0 ) roofTexSpanX = 10.0;
            roofTexSpanY = roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : 10.0;
            if ( roofTexSpanY <= 0.0 ) roofTexSpanY = 10.0;
        }
    }

	// prep for wall texture coordinate generation.
    double texWidthM  = wallSkin ? *wallSkin->imageWidth() : 0.0;
    double texHeightM = wallSkin ? *wallSkin->imageHeight() : 1.0;

    ConstGeometryIterator iter( input );
    while( iter.hasMore() )
    {
        const Geometry* part = iter.next();

        // skip a part that's too small
        if (part->size() < 2)
            continue;

        // add a new wall.
        structure.elevations.push_back(Elevation());
        Elevation& elevation = structure.elevations.back();

        double maxHeight = targetLen - minLoc.z();

        // Adjust the texture height so it is a multiple of the maximum height
        double div = osg::round(maxHeight / texHeightM);
        elevation.texHeightAdjustedM = div > 0.0 ? maxHeight / div : maxHeight;

        // Step 1 - Create the real corners and transform them into our target SRS.
        Corners corners;
        for(Geometry::const_iterator m = part->begin(); m != part->end(); ++m)
        {
            Corners::iterator corner = corners.insert(corners.end(), Corner());
            
            // mark as "from source", as opposed to being inserted by the algorithm.
            corner->isFromSource = true;
            corner->base = *m;

            // extrude:
            if ( height >= 0 ) // extrude up
            {
                if ( flatten )
                    corner->roof.set( corner->base.x(), corner->base.y(), targetLen );
                else
                    corner->roof.set( corner->base.x(), corner->base.y(), corner->base.z() + height );
            }
            else // height < 0 .. extrude down
            {
                corner->roof = *m;
                corner->base.z() += height;
            }
            
            // figure out the rooftop texture coords before doing any transformation:
			if (have_roof_image)
			{
				//calc uv from roof_image_name
				if (RoofHelper->ImageOK())
				{
					RoofHelper->LL2UV(corner->roof, corner->roofTexU, corner->roofTexV);
				}
			}
            else if ( roofSkin && srs )
            {
                double xr, yr;

                if ( srs && srs->isGeographic() && roofProjSRS )
                {
                    osg::Vec3d projRoofPt;
                    srs->transform( corner->roof, roofProjSRS.get(), projRoofPt );
                    xr = (projRoofPt.x() - roofBounds.xMin());
                    yr = (projRoofPt.y() - roofBounds.yMin());
                }
                else
                {
                    xr = (corner->roof.x() - roofBounds.xMin());
                    yr = (corner->roof.y() - roofBounds.yMin());
                }

                corner->roofTexU = (cosR*xr - sinR*yr) / roofTexSpanX;
                corner->roofTexV = (sinR*xr + cosR*yr) / roofTexSpanY;
            }

            // transform into target SRS.
            if (srs)
            {
                transformAndLocalize( corner->base, srs, corner->base, mapSRS, _world2local, makeECEF );
                transformAndLocalize( corner->roof, srs, corner->roof, mapSRS, _world2local, makeECEF );
            }

            // cache the length for later use.
            corner->height = (corner->roof - corner->base).length();
        }

        // Step 2 - Insert intermediate Corners as needed to satify texturing
        // requirements (if necessary) and record each corner offset (horizontal distance
        // from the beginning of the part geometry to the corner.)
        double cornerOffset    = 0.0;
        double nextTexBoundary = texWidthM;

        for(Corners::iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::iterator this_corner = c;

            Corners::iterator next_corner = c;
			bool isLastEdge = false;
			if ( ++next_corner == corners.end() )
			{
				isLastEdge = true;
				next_corner = corners.begin();
			}

            osg::Vec3d base_vec = next_corner->base - this_corner->base;
            double span = base_vec.length();

            this_corner->offsetX = cornerOffset;

            if (wallSkin)
            {
                base_vec /= span; // normalize
                osg::Vec3d roof_vec = next_corner->roof - this_corner->roof;
                roof_vec.normalize();

                while(nextTexBoundary < cornerOffset+span)
                {
                    // insert a new fake corner.
					Corners::iterator new_corner;

                    if ( isLastEdge )
                    {
						corners.push_back(Corner());
						new_corner = c;
						new_corner++;
                    }
                    else
                    {
						new_corner = corners.insert(next_corner, Corner());
					}

                    new_corner->isFromSource = false;
                    double advance = nextTexBoundary-cornerOffset;
                    new_corner->base = this_corner->base + base_vec*advance;
                    new_corner->roof = this_corner->roof + roof_vec*advance;
                    new_corner->height = (new_corner->roof - new_corner->base).length();
                    new_corner->offsetX = cornerOffset + advance;
                    nextTexBoundary += texWidthM;

                    // advance the main iterator
                    c = new_corner;
                }
            }

            cornerOffset += span;
        }

        // Step 3 - Calculate the angle of each corner.
        osg::Vec3d prev_vec;
        for(Corners::iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::const_iterator this_corner = c;

            Corners::const_iterator next_corner = c;
            if ( ++next_corner == corners.end() )
                next_corner = corners.begin();

            if ( this_corner == corners.begin() )
            {
                Corners::const_iterator prev_corner = corners.end();
                --prev_corner;
                prev_vec = this_corner->roof - prev_corner->roof;
                prev_vec.normalize();
            }

            osg::Vec3d this_vec = next_corner->roof - this_corner->roof;
            this_vec.normalize();
            if ( c != corners.begin() )
            {
                c->cosAngle = prev_vec * this_vec;
            }
        }

        // Step 4 - Create faces connecting each pair of Posts.
        Faces& faces = elevation.faces;
        for(Corners::const_iterator c = corners.begin(); c != corners.end(); ++c)
        {
            Corners::const_iterator this_corner = c;

            Corners::const_iterator next_corner = c;
            if ( ++next_corner == corners.end() )
                next_corner = corners.begin();
            
            // only close the shape for polygons.
            if (next_corner != corners.begin() || structure.isPolygon)
            {
                faces.push_back(Face());
                Face& face = faces.back();
                face.left  = *this_corner;
                face.right = *next_corner;

                // recalculate the final offset on the last face
                if ( next_corner == corners.begin() )
                {
                    osg::Vec3d vec = next_corner->roof - this_corner->roof;
                    face.right.offsetX = face.left.offsetX + vec.length();
                }

                face.widthM = next_corner->offsetX - this_corner->offsetX;
            }
        }
    }

    return true;
}

bool
ExtrudeGeometryFilter::LoadStructure(Geometry* multiinput,
    double                  height,
    bool                    flatten,
    float                   verticalOffset,
    const SkinResource*     wallSkin,
    const SkinResource*     roofSkin,
    GeoStructure&           structure,
    FilterContext&          cx,
    bool					have_roof_image,
    std::string			    roof_image_name,
    bool					roof_image_has_mat_index,
    std::string			    roof_mat_index_name,
    ExtrudeGeomGDALHelper* RoofHelper)
{
#ifdef _DEBUG
    int fubar = 0;
#endif
    // whether this is a closed polygon structure.
    structure.isPolygon = true;

    // store the vert offset for later encoding
    structure.verticalOffset = verticalOffset;

#if 0
    double texWidthM = wallSkin ? *wallSkin->imageWidth() : 1.0;
    double texHeightM = wallSkin ? *wallSkin->imageHeight() : 1.0;

    // Wall Scale and bias:
    osg::Vec2f scale, bias;
    float layer;
    if (wallSkin)
    {
        bias.set(wallSkin->imageBiasS().get(), wallSkin->imageBiasT().get());
        scale.set(wallSkin->imageScaleS().get(), wallSkin->imageScaleT().get());
        layer = (float)wallSkin->imageLayer().get();
    }

    bool     tex_repeats_y = wallSkin && wallSkin->isTiled() == true;
#endif

    double maxWallLen = 0.0;
    double targetLen = -DBL_MAX;
    GeometryIterator iter(multiinput, false);
    while (iter.hasMore())
    {
        Geometry* input = iter.next();
        if (input->getType() == Geometry::TYPE_POLYGON)
        { 
            // extrusion working variables
            osg::Vec3d minLoc(DBL_MAX, DBL_MAX, DBL_MAX);
            double     minLoc_len = DBL_MAX;
            osg::Vec3d maxLoc(0, 0, 0);
            double     maxLoc_len = 0;

            // Initial pass over the geometry does two things:
            // 1: Calculate the minimum Z across all parts.
            // 2: Establish a "target length" for extrusion

            int pcount = 0;
            osg::Vec3d P0;
            osg::Vec2d P02;
            osg::Vec3d P1;
            osg::Vec3d P2;
            bool isWall = false;
            bool isRoof = false;
            GeoFace CurFace;
            for (Geometry::const_iterator m = input->begin(); m != input->end(); ++m)
            {
                osg::Vec3d point;
                transformAndLocalize(*m, structure.srs, point, structure.mapSRS, _world2local, structure.makeECEF);
                GeoPoint gpoint(point);
                if(abs(gpoint.pLoc.z() < VERTICALZMAX))
                    gpoint.pLoc.z() = 0.0;
                CurFace.Points.push_back(gpoint);
                if (pcount == 0)
                {
                    P0 = point;
                    P02.x() = P0.x();
                    P02.y() = P0.y();
                }
                else if(pcount == 1)
                {
                    P1 = point;
                }
                else if (pcount == 2)
                {
                    P2 = point;
                    osg::Vec3d Norm;
                    osg::Vec4d PlnC = PlanerConstants(P0, P1, P2, Norm);
                    if (abs(Norm.z()) <= VERTICALZMAX)
                    {
                        isWall = true;
                        isRoof = false;
                    }
                    else
                    {
                        isWall = false;
                        isRoof = true;
                    }
                    CurFace.PlanerC = PlnC;
                }

                if (point.z() > targetLen)
                    targetLen = point.z();

                if (point.z() < minLoc.z())
                    minLoc = point;

                if (point.z() > maxLoc.z())
                    maxLoc = point;
                if (pcount > 0)
                {
                    osg::Vec2d P2B(point.x(), point.y());
                    double Len = (P02 - P2B).length();
                    if(Len > maxWallLen)
                        maxWallLen = Len;
                }
                ++pcount;
            }

            if(maxLoc.z() <= DBL_EPSILON) //This should be a floor polygon
                isRoof = false;

            // apply the height offsets
            //height    -= heightOffset;
            //targetLen -= heightOffset;

            float   roofRotation = 0.0f;
            Bounds  roofBounds;
            float   sinR = 0.0f, cosR = 0.0f;
            double  roofTexSpanX = 0.0, roofTexSpanY = 0.0;
            osg::ref_ptr<const SpatialReference> roofProjSRS;
            if(isRoof)
            {
                structure.Roofs.push_back(CurFace);
            }
            else if(isWall)
            {
                structure.Walls.push_back(CurFace);
            }

            if ((roofSkin || have_roof_image) && isRoof)
            {
                GeoFace& thisRoof = structure.Roofs.back();
                roofBounds = thisRoof.getBounds();

                // if our data is lat/long, we need to reproject the geometry and the bounds into a projected
                // coordinate system in order to properly generate tex coords.
                roofRotation = getApparentRotation(input);

                sinR = sin(roofRotation);
                cosR = cos(roofRotation);

                if(roofSkin)
                {
                    if (!roofSkin->isTiled().value())
                    {
                        //note: non-tiled roofs don't really work atm.
                        roofTexSpanX = cosR * roofBounds.width() - sinR * roofBounds.height();
                        roofTexSpanY = sinR * roofBounds.width() + cosR * roofBounds.height();
                    }
                    else
                    {
                        roofTexSpanX = roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : 10.0;
                        if (roofTexSpanX <= 0.0) roofTexSpanX = 10.0;
                        roofTexSpanY = roofSkin->imageHeight().isSet() ? *roofSkin->imageHeight() : roofSkin->imageWidth().isSet() ? *roofSkin->imageWidth() : 10.0;
                        if (roofTexSpanY <= 0.0) roofTexSpanY = 10.0;
                    }
                }
                int pcount = 0;
                for (Geometry::const_iterator m = input->begin(); m != input->end(); ++m)
                {
                    osg::Vec3d point = *m;

                    // figure out the rooftop texture coords before doing any transformation:
                    if (have_roof_image)
                    {
                        //calc uv from roof_image_name
                        if (RoofHelper->ImageOK())
                        {
                            RoofHelper->LL2UV(point, thisRoof.Points[pcount].TexU, thisRoof.Points[pcount].TexV);
                        }
                    }
                    //                  else if (roofSkin && srs)
                    else if (roofSkin)
                    {
                        double xr, yr;

                        xr = (thisRoof.Points[pcount].pLoc.x() - roofBounds.xMin());
                        yr = (thisRoof.Points[pcount].pLoc.y() - roofBounds.yMin());
                    
                        thisRoof.Points[pcount].TexU = (float)(((cosR * xr - sinR * yr) / roofTexSpanX) + 0.5);
                        thisRoof.Points[pcount].TexV = (float)(((sinR * xr + cosR * yr) / roofTexSpanY) + 0.5);
                    }
                    ++pcount;
                }
            }
        }
    }

    if (wallSkin)
    {
        for(GeoFaces::iterator g = structure.Walls.begin(); g != structure.Walls.end(); ++g)
        {
            GeoPoints::iterator p0 = g->Points.begin();
            for(GeoPoints::iterator p = g->Points.begin(); p != g->Points.end(); ++p)
            {
                // prep for wall texture coordinate generation.
//              double texWidthM = wallSkin ? *wallSkin->imageWidth() : 0.0;
//              double texHeightM = wallSkin ? *wallSkin->imageHeight() : 1.0;

//              double maxHeight = targetLen - minLoc.z();

                // Adjust the texture height so it is a multiple of the maximum height
//              double div = osg::round(maxHeight / texHeightM);
//              double texHeightAdjustedM = div > 0.0 ? maxHeight / div : maxHeight;
                osg::Vec2d P01;
                P01.x() = p0->pLoc.x();
                P01.y() = p0->pLoc.y();
                osg::Vec2d P02;
                P02.x() = p->pLoc.x();
                P02.y() = p->pLoc.y();
                double Ud = (P02 - P01).length();
                p->TexU = Ud / maxWallLen;
                p->TexV = p->pLoc.z() / targetLen;
            }
        }
    }

    return true;
}

osg::Vec4d ExtrudeGeometryFilter::PlanerConstants(osg::Vec3d& P0, osg::Vec3d& P1, osg::Vec3d& P2, osg::Vec3d &Norm)
{
    osg::Vec4d PlnC;
    double a1 = P1.x() - P0.x();
    double b1 = P1.y() - P0.y();
    double c1 = P1.z() - P0.z();

    double a2 = P2.x() - P1.x();
    double b2 = P2.y() - P1.y();
    double c2 = P2.z() - P1.z();

    PlnC.x() = b1 * c2 - b2 * c1; //A
    PlnC.y() = a2 * c1 - a1 * c2; //B
    PlnC.z() = a1 * b2 - b1 * a2; //C
    PlnC.w() = (-PlnC.x() * P0.x() - PlnC.y() * P0.y() - PlnC.z() * P0.z()); //D
    Norm.x() = PlnC.x();
    Norm.y() = PlnC.y();
    Norm.z() = PlnC.z();
    Norm.normalize();
#if 0

    int fubar = 0;
    double d0 = (PlnC.x() * P0.x()) + (PlnC.y() * P0.y()) + (PlnC.z() * P0.z()) + PlnC.w();
    if(d0 > DBL_EPSILON)
        ++fubar;

    double d1 = (PlnC.x() * P1.x()) + (PlnC.y() * P1.y()) + (PlnC.z() * P1.z()) + PlnC.w();
    if (d1 > DBL_EPSILON)
        ++fubar;

    double d2 = (PlnC.x() * P2.x()) + (PlnC.y() * P2.y()) + (PlnC.z() * P2.z()) + PlnC.w();
    if (d2 > DBL_EPSILON)
        ++fubar;

#endif
    return PlnC;
}

bool
ExtrudeGeometryFilter::buildWallGeometry(const Structure&     structure,
                                         osg::Geometry*       walls,
                                         const osg::Vec4&     wallColor,
                                         const osg::Vec4&     wallBaseColor,
                                         const SkinResource*  wallSkin)
{
    bool madeGeom = true;

    // 6 verts per face total (3 triangles)
    unsigned numWallVerts = 6 * structure.getNumPoints();

    double texWidthM   = wallSkin ? *wallSkin->imageWidth()  : 1.0;
    double texHeightM  = wallSkin ? *wallSkin->imageHeight() : 1.0;
    bool   useColor    = (!wallSkin || wallSkin->texEnvMode() != osg::TexEnv::DECAL) && !_makeStencilVolume;
    
    // Scale and bias:
    osg::Vec2f scale, bias;
    float layer;
    if ( wallSkin )
    {
        bias.set (wallSkin->imageBiasS().get(),  wallSkin->imageBiasT().get());
        scale.set(wallSkin->imageScaleS().get(), wallSkin->imageScaleT().get());
        layer = (float)wallSkin->imageLayer().get();
    }

    // create all the OSG geometry components
    osg::Vec3Array* verts = new osg::Vec3Array( numWallVerts );
    walls->setVertexArray( verts );
    
    osg::Vec3Array* tex = 0L;
    if ( wallSkin )
    { 
        tex = new osg::Vec3Array( numWallVerts );
        walls->setTexCoordArray( 0, tex );
		if (wallSkin->materialURI().isSet())
		{
			walls->setTexCoordArray(1, tex);
		}
		if (wallSkin->SurfaceMaterialCode().isSet())
		{
			__int16 surface = (short)wallSkin->SurfaceMaterialCode().value();
			__int16 fid = 180; //General Building
			walls->setUserValue("<UA:SMC>", surface);
			walls->setUserValue("<UA:FID>", fid);
//			static osg::ref_ptr<osg::CullFace> cullFace = new osg::CullFace(osg::CullFace::BACK);
		}
	}

    osg::Vec4Array* colors = 0L;
    if ( useColor )
    {
        colors = new osg::Vec4Array( numWallVerts );
        walls->setColorArray( colors );
        walls->setColorBinding( osg::Geometry::BIND_PER_VERTEX );
    }

    osg::Vec4Array* anchors = 0L;
    
    // If GPU clamping is in effect, create clamping attributes.
    if ( _gpuClamping )
    {
        anchors = new osg::Vec4Array( numWallVerts );
        walls->setVertexAttribArray    ( Clamping::AnchorAttrLocation, anchors );
        walls->setVertexAttribBinding  ( Clamping::AnchorAttrLocation, osg::Geometry::BIND_PER_VERTEX );
        walls->setVertexAttribNormalize( Clamping::AnchorAttrLocation, false );
    }

    unsigned vertptr = 0;
    bool     tex_repeats_y = wallSkin && wallSkin->isTiled() == true;

    bool flatten =
        _style.has<ExtrusionSymbol>() &&
        _style.get<ExtrusionSymbol>()->flatten() == true;

    for(Elevations::const_iterator elev = structure.elevations.begin(); elev != structure.elevations.end(); ++elev)
    {
        osg::DrawElements* de = 
            numWallVerts > 0xFFFF ? (osg::DrawElements*) new osg::DrawElementsUInt  ( GL_TRIANGLES ) :
            numWallVerts > 0xFF   ? (osg::DrawElements*) new osg::DrawElementsUShort( GL_TRIANGLES ) :
                                    (osg::DrawElements*) new osg::DrawElementsUByte ( GL_TRIANGLES );

        // pre-allocate for speed
        de->reserveElements( numWallVerts );

        walls->addPrimitiveSet( de );

        for(Faces::const_iterator f = elev->faces.begin(); f != elev->faces.end(); ++f, vertptr+=6)
        {
            // set the 6 wall verts.
            (*verts)[vertptr+0] = f->left.roof;
            (*verts)[vertptr+1] = f->left.base;
            (*verts)[vertptr+2] = f->right.base;
            (*verts)[vertptr+3] = f->right.base;
            (*verts)[vertptr+4] = f->right.roof;
            (*verts)[vertptr+5] = f->left.roof;
            
            if ( anchors )
            {
                float x = structure.baseCentroid.x(), y = structure.baseCentroid.y(), vo = structure.verticalOffset;

                (*anchors)[vertptr+1].set( x, y, vo, Clamping::ClampToGround );
                (*anchors)[vertptr+2].set( x, y, vo, Clamping::ClampToGround );
                (*anchors)[vertptr+3].set( x, y, vo, Clamping::ClampToGround );

                if ( flatten )
                {
                    (*anchors)[vertptr+0].set( x, y, vo, Clamping::ClampToAnchor );
                    (*anchors)[vertptr+4].set( x, y, vo, Clamping::ClampToAnchor );
                    (*anchors)[vertptr+5].set( x, y, vo, Clamping::ClampToAnchor );
                }
                else
                {                    
                    (*anchors)[vertptr+0].set( x, y, vo + f->left.height,  Clamping::ClampToGround );
                    (*anchors)[vertptr+4].set( x, y, vo + f->right.height, Clamping::ClampToGround );
                    (*anchors)[vertptr+5].set( x, y, vo + f->left.height,  Clamping::ClampToGround );
                }
            }

            // Assign wall polygon colors.
            if (useColor)
            {
                (*colors)[vertptr+0] = wallColor;
                (*colors)[vertptr+1] = wallBaseColor;
                (*colors)[vertptr+2] = wallBaseColor;
                (*colors)[vertptr+3] = wallBaseColor;
                (*colors)[vertptr+4] = wallColor;
                (*colors)[vertptr+5] = wallColor;
            }

            // Calculate texture coordinates:
            if (wallSkin)
            {
                // Calculate left and right corner V coordinates:
                double hL = tex_repeats_y ? (f->left.roof - f->left.base).length()   : elev->texHeightAdjustedM;
                double hR = tex_repeats_y ? (f->right.roof - f->right.base).length() : elev->texHeightAdjustedM;
                
                // Calculate the texture coordinates at each corner. The structure builder
                // will have spaced the verts correctly for this to work.
                float uL = fmod( f->left.offsetX, texWidthM ) / texWidthM;
                float uR = fmod( f->right.offsetX, texWidthM ) / texWidthM;

                // Correct for the case in which the rightmost corner is exactly on a
                // texture boundary.
                if ( uR < uL || (uL == 0.0 && uR == 0.0))
                    uR = 1.0f;

                osg::Vec2f texBaseL( uL, 0.0f );
                osg::Vec2f texBaseR( uR, 0.0f );
                osg::Vec2f texRoofL( uL, hL/elev->texHeightAdjustedM );
                osg::Vec2f texRoofR( uR, hR/elev->texHeightAdjustedM );

                texRoofL = bias + osg::componentMultiply(texRoofL, scale);
                texRoofR = bias + osg::componentMultiply(texRoofR, scale);
                texBaseL = bias + osg::componentMultiply(texBaseL, scale);
                texBaseR = bias + osg::componentMultiply(texBaseR, scale);

                (*tex)[vertptr+0].set( texRoofL.x(), texRoofL.y(), layer );
                (*tex)[vertptr+1].set( texBaseL.x(), texBaseL.y(), layer );
                (*tex)[vertptr+2].set( texBaseR.x(), texBaseR.y(), layer );
                (*tex)[vertptr+3].set( texBaseR.x(), texBaseR.y(), layer );
                (*tex)[vertptr+4].set( texRoofR.x(), texRoofR.y(), layer );
                (*tex)[vertptr+5].set( texRoofL.x(), texRoofL.y(), layer );
            }
#if 0
            for(int i=0; i<6; ++i)
            {
                de->addElement( vertptr+i );
            }
#endif
			for (int i = 5; i >= 0; --i)
			{
				de->addElement(vertptr + i);
			}
        }
    }
    
    // generate per-vertex normals, altering the geometry as necessary to avoid
    // smoothing around sharp corners

    // TODO: reconsider this, given the new Structure setup
    // it won't actual smooth corners since we don't have shared edges.
	//GAJ Normals comming out reversed
	walls->getOrCreateStateSet()->setAttributeAndModes(new osg::FrontFace(osg::FrontFace::COUNTER_CLOCKWISE), osg::StateAttribute::ON);
//	walls->getOrCreateStateSet()->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON);
	
	osgUtil::SmoothingVisitor::smooth(
        *walls,
        osg::DegreesToRadians(_wallAngleThresh_deg) );

	osg::Array* normal = walls->getNormalArray();
	int size = normal->getNumElements();
	osg::Vec3Array * v3d = dynamic_cast <osg::Vec3Array*>(normal);
#if 0
	for (int i = 0; i < size; ++i)
	{
		(*v3d)[i][0] *= -1.0;
		(*v3d)[i][1] *= -1.0;
		(*v3d)[i][2] *= -1.0;
	}
#endif
    return madeGeom;
}

bool
ExtrudeGeometryFilter::FormWallGeometry(GeoStructure& structure,
                                        osg::Geometry* walls,
                                        const osg::Vec4& wallColor,
                                        const osg::Vec4& wallBaseColor,
                                        const SkinResource* wallSkin)
{
    osg::Vec3Array* verts = new osg::Vec3Array();
    walls->setVertexArray(verts);

    osg::Vec4Array* color = new osg::Vec4Array();
    walls->setColorArray(color);
    walls->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::Vec3Array* tex = 0L;
    if (wallSkin)
    {
        tex = new osg::Vec3Array();
        walls->setTexCoordArray(0, tex);
        if ((wallSkin && wallSkin->materialURI().isSet()))
        {
            walls->setTexCoordArray(1, tex);
        }
    }
    if (wallSkin)
    {
        if (wallSkin->SurfaceMaterialCode().isSet())
        {
            __int16 surface = (short)wallSkin->SurfaceMaterialCode().value();
            __int16 fid = 180; //General Building
            walls->setUserValue("<UA:SMC>", surface);
            walls->setUserValue("<UA:FID>", fid);
        }
    }

    //Remove redundant points;
    for (GeoFaces::iterator g = structure.Walls.begin(); g != structure.Walls.end(); ++g)
    {
        GeoPoints::iterator p = g->Points.end() - 1;
        g->Points.erase(p);
    }

    // Create a series of line loops that the tessellator can reorganize
    // into polygons.
    unsigned vertptr = 0;
    bool need2tesselate = false;
    for (GeoFaces::const_iterator g = structure.Walls.begin(); g != structure.Walls.end(); ++g)
    {
        unsigned elevptr = vertptr;
        if (g->Points.size() == 4)
        {
            //Just add the triangles
            verts->push_back(g->Points[1].pLoc);
            color->push_back(wallColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(g->Points[1].TexU, g->Points[1].TexV, (float)0.0f));
            }
            ++vertptr;
            verts->push_back(g->Points[0].pLoc);
            color->push_back(wallColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(g->Points[0].TexU, g->Points[0].TexV, (float)0.0f));
            }
            ++vertptr;
            verts->push_back(g->Points[3].pLoc);
            color->push_back(wallColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(g->Points[3].TexU, g->Points[3].TexV, (float)0.0f));
            }
            ++vertptr;
            verts->push_back(g->Points[3].pLoc);
            color->push_back(wallColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(g->Points[3].TexU, g->Points[3].TexV, (float)0.0f));
            }
            ++vertptr;
            verts->push_back(g->Points[2].pLoc);
            color->push_back(wallColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(g->Points[2].TexU, g->Points[2].TexV, (float)0.0f));
            }
            ++vertptr;
            verts->push_back(g->Points[1].pLoc);
            color->push_back(wallColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(g->Points[1].TexU, g->Points[1].TexV, (float)0.0f));
            }
            ++vertptr;
            walls->addPrimitiveSet(new osg::DrawArrays(GL_TRIANGLES, elevptr, vertptr - elevptr));

        }
        else
        { 
            need2tesselate = true;
            for (GeoPoints::const_iterator p = g->Points.begin(); p != g->Points.end(); ++p)
            {
                // Only use source verts; we skip interim verts inserted by the 
                // structure building since they are co-linear anyway and thus we don't
                // need them for the roof line.
                verts->push_back(p->pLoc);
                color->push_back(wallColor);
                if (tex)
                {
                    tex->push_back(osg::Vec3f(p->TexU, p->TexV, (float)0.0f));
                }
                ++vertptr;
            }
            walls->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, elevptr, vertptr - elevptr));
        }
    }

    walls->getOrCreateStateSet()->setAttributeAndModes(new osg::FrontFace(osg::FrontFace::COUNTER_CLOCKWISE), osg::StateAttribute::ON);

    osgUtil::SmoothingVisitor::smooth(
        *walls,
        osg::DegreesToRadians(_wallAngleThresh_deg));


    // Tessellate the roof lines into polygons.
    if(need2tesselate)
    {
        osgUtil::Tessellator tess;
        tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
        tess.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
        tess.retessellatePolygons(*walls);
    }

    return true;
}


bool
ExtrudeGeometryFilter::buildRoofGeometry(const Structure&     structure,
                                         osg::Geometry*       roof,
                                         const osg::Vec4&     roofColor,
                                         const SkinResource*  roofSkin,
										 const bool			  have_roof_image,
										 const bool			  have_roof_mat_index)
{    
    osg::Vec3Array* verts = new osg::Vec3Array();
    roof->setVertexArray( verts );

    osg::Vec4Array* color = new osg::Vec4Array();
    roof->setColorArray( color );
    roof->setColorBinding( osg::Geometry::BIND_PER_VERTEX );

    osg::Vec3Array* tex = 0L;
    if ( roofSkin || have_roof_image)
    {
        tex = new osg::Vec3Array();
        roof->setTexCoordArray(0, tex);
		if ((roofSkin && roofSkin->materialURI().isSet()) || have_roof_mat_index)
		{
			roof->setTexCoordArray(1, tex);
		}
    }
	if (roofSkin)
	{
		if (roofSkin->SurfaceMaterialCode().isSet())
		{
			__int16 surface = (short)roofSkin->SurfaceMaterialCode().value();
			__int16 fid = 180; //General Building
			roof->setUserValue("<UA:SMC>", surface);
			roof->setUserValue("<UA:FID>", fid);
		}
	}
    osg::Vec4Array* anchors = 0L;    
    if ( _gpuClamping )
    {
        // fake out the OSG tessellator. It does not preserve attrib arrays in the Tessellator.
        // so we will put them in one of the texture arrays and copy them to an attrib array 
        // after tessellation. #osghack
        anchors = new osg::Vec4Array();
        roof->setTexCoordArray(1, anchors);
    }

    bool flatten =
        _style.has<ExtrusionSymbol>() &&
        _style.get<ExtrusionSymbol>()->flatten() == true;

    // Create a series of line loops that the tessellator can reorganize
    // into polygons.
    unsigned vertptr = 0;
    for(Elevations::const_iterator e = structure.elevations.begin(); e != structure.elevations.end(); ++e)
    {
        unsigned elevptr = vertptr;
        for(Faces::const_iterator f = e->faces.begin(); f != e->faces.end(); ++f)
        {
            // Only use source verts; we skip interim verts inserted by the 
            // structure building since they are co-linear anyway and thus we don't
            // need them for the roof line.
            if ( f->left.isFromSource )
            {
                verts->push_back( f->left.roof );
                color->push_back( roofColor );

                if ( tex )
                {
                    tex->push_back( osg::Vec3f(f->left.roofTexU, f->left.roofTexV, (float)0.0f) );
                }

                if ( anchors )
                {
                    float 
                        x = structure.baseCentroid.x(),
                        y = structure.baseCentroid.y(), 
                        vo = structure.verticalOffset;

                    if ( flatten )
                    {
                        anchors->push_back( osg::Vec4f(x, y, vo, Clamping::ClampToAnchor) );
                    }
                    else
                    {
                        anchors->push_back( osg::Vec4f(x, y, vo + f->left.height, Clamping::ClampToGround) );
                    }
                }
                ++vertptr;
            }
        }
        roof->addPrimitiveSet( new osg::DrawArrays(GL_LINE_LOOP, elevptr, vertptr-elevptr) );
    } 

    osg::Vec3Array* normal = new osg::Vec3Array(verts->size());
    roof->setNormalArray( normal );
    roof->setNormalBinding( osg::Geometry::BIND_PER_VERTEX );
    normal->assign( verts->size(), osg::Vec3(0,0,1) );

    int v = verts->size();

    // Tessellate the roof lines into polygons.
    osgEarth::Tessellator oeTess;
    if (!oeTess.tessellateGeometry(*roof))
    {
        //fallback to osg tessellator
        OE_DEBUG << LC << "Falling back on OSG tessellator (" << roof->getName() << ")" << std::endl;

        osgUtil::Tessellator tess;
        tess.setTessellationType( osgUtil::Tessellator::TESS_TYPE_GEOMETRY );
        tess.setWindingType( osgUtil::Tessellator::TESS_WINDING_ODD );
        tess.retessellatePolygons( *roof );
    }

    // Move the anchors to the correct place. :)
    if ( _gpuClamping )
    {
        osg::Vec4Array* a = static_cast<osg::Vec4Array*>(roof->getTexCoordArray(1));
        if ( a )
        {
            roof->setVertexAttribArray    ( Clamping::AnchorAttrLocation, a );
            roof->setVertexAttribBinding  ( Clamping::AnchorAttrLocation, osg::Geometry::BIND_PER_VERTEX );
            roof->setVertexAttribNormalize( Clamping::AnchorAttrLocation, false );
            roof->setTexCoordArray(1, 0L);
        }
    }

    return true;
}

bool
ExtrudeGeometryFilter::FormRoofGeometry(GeoStructure& structure,
                                        osg::Geometry* roof,
                                        const osg::Vec4& roofColor,
                                        const SkinResource* roofSkin,
                                        const bool have_roof_image,
                                        const bool have_roof_mat_index)
{
    osg::Vec3Array* verts = new osg::Vec3Array();
    roof->setVertexArray(verts);

    osg::Vec4Array* color = new osg::Vec4Array();
    roof->setColorArray(color);
    roof->setColorBinding(osg::Geometry::BIND_PER_VERTEX);

    osg::Vec3Array* tex = 0L;
    if (roofSkin || have_roof_image)
    {
        tex = new osg::Vec3Array();
        roof->setTexCoordArray(0, tex);
        if ((roofSkin && roofSkin->materialURI().isSet()) || have_roof_mat_index)
        {
            roof->setTexCoordArray(1, tex);
        }
    }
    if (roofSkin)
    {
        if (roofSkin->SurfaceMaterialCode().isSet())
        {
            __int16 surface = (short)roofSkin->SurfaceMaterialCode().value();
            __int16 fid = 180; //General Building
            roof->setUserValue("<UA:SMC>", surface);
            roof->setUserValue("<UA:FID>", fid);
        }
    }

    bool flatten =
        _style.has<ExtrusionSymbol>() &&
        _style.get<ExtrusionSymbol>()->flatten() == true;

    //Remove redundant points;
    for (GeoFaces::iterator g = structure.Roofs.begin(); g != structure.Roofs.end(); ++g)
    {
        GeoPoints::iterator p = g->Points.end() - 1;
        g->Points.erase(p);
    }

    // Create a series of line loops that the tessellator can reorganize
    // into polygons.
    unsigned vertptr = 0;
    for(GeoFaces::const_iterator g = structure.Roofs.begin(); g != structure.Roofs.end(); ++g)
    {
        unsigned elevptr = vertptr;
        for(GeoPoints::const_iterator p = g->Points.begin(); p != g->Points.end(); ++p)
        {
            // Only use source verts; we skip interim verts inserted by the 
            // structure building since they are co-linear anyway and thus we don't
            // need them for the roof line.
            verts->push_back(p->pLoc);
            color->push_back(roofColor);
            if (tex)
            {
                tex->push_back(osg::Vec3f(p->TexU, p->TexV, (float)0.0f));
            }
            ++vertptr;
        }
        roof->addPrimitiveSet(new osg::DrawArrays(GL_LINE_LOOP, elevptr, vertptr - elevptr));
    }


    osgUtil::SmoothingVisitor::smooth(
        *roof,
        osg::DegreesToRadians(_wallAngleThresh_deg));


    // Tessellate the roof lines into polygons.

    osgUtil::Tessellator tess;
    tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
    tess.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
    tess.retessellatePolygons(*roof);

    return true;
}

bool
ExtrudeGeometryFilter::buildOutlineGeometry(const Structure&  structure,
                                            osg::Geometry*    outline,
                                            const osg::Vec4&  outlineColor,
                                            float             minCreaseAngleDeg)
{
    // minimum angle between adjacent faces for which to draw a post.
    const float cosMinAngle = cos(osg::DegreesToRadians(minCreaseAngleDeg));

    osg::Vec3Array* verts = new osg::Vec3Array();
    outline->setVertexArray( verts );

    osg::Vec4Array* color = new osg::Vec4Array();
    outline->setColorArray( color );
    outline->setColorBinding( osg::Geometry::BIND_OVERALL );
    color->push_back( outlineColor );

    osg::DrawElements* de = new osg::DrawElementsUInt(GL_LINES);
    outline->addPrimitiveSet(de);
        
    osg::Vec4Array* anchors = 0L;
    if ( _gpuClamping )
    {
        anchors = new osg::Vec4Array();
        outline->setVertexAttribArray    ( Clamping::AnchorAttrLocation, anchors );
        outline->setVertexAttribBinding  ( Clamping::AnchorAttrLocation, osg::Geometry::BIND_PER_VERTEX );
        outline->setVertexAttribNormalize( Clamping::AnchorAttrLocation, false );
    }

    bool flatten =
        _style.has<ExtrusionSymbol>() &&
        _style.get<ExtrusionSymbol>()->flatten() == true;
    
    float
        x  = structure.baseCentroid.x(),
        y  = structure.baseCentroid.y(),
        vo = structure.verticalOffset;

    unsigned vertptr = 0;
    for(Elevations::const_iterator e = structure.elevations.begin(); e != structure.elevations.end(); ++e)
    {
        osg::Vec3d prev_vec;
        unsigned elevptr = vertptr;
        for(Faces::const_iterator f = e->faces.begin(); f != e->faces.end(); ++f)
        {
            // Only use source verts for posts.
            bool drawPost     = f->left.isFromSource;
            bool drawCrossbar = true;

            osg::Vec3d this_vec = f->right.roof - f->left.roof;
            this_vec.normalize();

            if (f->left.isFromSource && f != e->faces.begin())
            {
                drawPost = (this_vec * prev_vec) < cosMinAngle;
            }

            if ( drawPost || drawCrossbar )
            {
                verts->push_back( f->left.roof );
                if ( anchors && flatten  ) anchors->push_back(osg::Vec4f(x, y, vo, Clamping::ClampToAnchor));
                if ( anchors && !flatten ) anchors->push_back(osg::Vec4f(x, y, vo + f->left.height, Clamping::ClampToGround));
            }

            if ( drawPost )
            {
                verts->push_back( f->left.base );
                if ( anchors ) anchors->push_back( osg::Vec4f(x, y, vo, Clamping::ClampToGround) );
                de->addElement(vertptr);
                de->addElement(verts->size()-1);
            }

            if ( drawCrossbar )
            {
                verts->push_back( f->right.roof );
                if ( anchors && flatten  ) anchors->push_back(osg::Vec4f(x, y, vo, Clamping::ClampToAnchor));
                if ( anchors && !flatten ) anchors->push_back(osg::Vec4f(x, y, vo + f->right.height, Clamping::ClampToGround));
                de->addElement(vertptr);
                de->addElement(verts->size()-1);
            }

            vertptr = verts->size();

            prev_vec = this_vec;
        }

        // Draw an end-post if this isn't a closed polygon.
        if ( !structure.isPolygon )
        {
            Faces::const_iterator last = e->faces.end()-1;
            verts->push_back( last->right.roof );
            if ( anchors && flatten  ) anchors->push_back(osg::Vec4f(x, y, vo, Clamping::ClampToAnchor));
            if ( anchors && !flatten ) anchors->push_back(osg::Vec4f(x, y, vo + last->right.height, Clamping::ClampToGround));
            de->addElement( verts->size()-1 );
            verts->push_back( last->right.base );
            if ( anchors ) anchors->push_back( osg::Vec4f(x, y, vo, Clamping::ClampToGround));
            de->addElement( verts->size()-1 );
        }
    }

    return true;
}

void
ExtrudeGeometryFilter::addDrawable(osg::Drawable*       drawable,
                                   osg::StateSet*       stateSet,
                                   const std::string&   name,
                                   Feature*             feature,
                                   FeatureIndexBuilder* index )
{
    // find the geode for the active stateset, creating a new one if necessary. NULL is a 
    // valid key as well.
    osg::Geode* geode = _geodes[stateSet].get();
    if ( !geode )
    {
        geode = new osg::Geode();
        geode->setStateSet( stateSet );
        _geodes[stateSet] = geode;
    }

    geode->addDrawable( drawable );

    if ( !name.empty() )
    {
        drawable->setName( name );
    }

    if ( index )
    {
        index->tagDrawable( drawable, feature );
    }
}

bool
ExtrudeGeometryFilter::process( FeatureList& features, FilterContext& context )
{
    // seed our random number generators
    if(_wallSkinSymbol.valid())
        _wallSkinPRNG.seed(*_wallSkinSymbol->randomSeed());
    else
        _wallSkinPRNG.seed(0);
    if(_roofSkinSymbol.valid())
        _roofSkinPRNG.seed(*_roofSkinSymbol->randomSeed());
    else
        _roofSkinPRNG.seed(0);

    for( FeatureList::iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();
        ExtrudeAFeature(input, context);
    }

    return true;
}

bool
ExtrudeGeometryFilter::ExtrudeAFeature(Feature * input, FilterContext& context)
{
#ifdef _DEBUG
    int fubar = 0;
#endif
    // run a symbol script if present.
    if (_extrusionSymbol->script().isSet())
    {
        StringExpression temp(_extrusionSymbol->script().get());
        input->eval(temp, &context);
    }

    bool roofTextureIsFromImage = false;
    std::string roofTextureImageName = "";
    bool roofTextureHasMatIndex = false;
    std::string roofTextureMatIndexName = "";
    ExtrudeGeomGDALHelper* RoofHelper = NULL;
    if (input->hasAttr("UseNameAsRoofTexture") && input->hasAttr("teximagename"))
    {
        if (input->getString("UseNameAsRoofTexture") == "true")
        {
            roofTextureImageName = input->getString("teximagename");
            roofTextureIsFromImage = true;
            if (input->hasAttr("MatIndexUrl"))
            {
                roofTextureHasMatIndex = true;
                roofTextureMatIndexName = input->getString("MatIndexUrl");
            }
        }
    }
    if (roofTextureIsFromImage)
    {
        RoofHelper = new ExtrudeGeomGDALHelper(roofTextureImageName);
        if (roofTextureHasMatIndex)
        {
            RoofHelper->Add_Mat_Index(roofTextureHasMatIndex, roofTextureMatIndexName);
        }
    }

    // iterator over the parts.
    GeometryIterator iter(input->getGeometry(), false);
    bool hasHoles = false;
    while (iter.hasMore())
    {
        Geometry* part = iter.next();

        osg::ref_ptr<osg::Geometry> walls = new osg::Geometry();

        osg::ref_ptr<osg::Geometry> rooflines = 0L;
        osg::ref_ptr<osg::Geometry> baselines = 0L;
        osg::ref_ptr<osg::Geometry> outlines = 0L;

        if (part->getType() == Geometry::TYPE_POLYGON)
        {
            rooflines = new osg::Geometry();

            // prep the shapes by making sure all polys are open:
            static_cast<Polygon*>(part)->open();
            if (static_cast<Polygon*>(part)->getHoles().size() > 0)
            {
                hasHoles = true;
            }
        }

        // fire up the outline geometry if we have a line symbol.
        if (_outlineSymbol != 0L)
        {
            outlines = new osg::Geometry();
        }

        // make a base cap if we're doing stencil volumes.
        if (_makeStencilVolume)
        {
            baselines = new osg::Geometry();
        }

        // calculate the extrusion height:
        float height;

        if (_heightCallback.valid())
        {
            height = _heightCallback->operator()(input, context);
        }
        else if (_heightExpr.isSet())
        {
            height = input->eval(_heightExpr.mutable_value(), &context);
        }
        else
        {
            height = *_extrusionSymbol->height();
        }

        osg::ref_ptr<osg::StateSet> wallStateSet;
        osg::ref_ptr<osg::StateSet> roofStateSet;

        // calculate the wall texturing:
        SkinResource* wallSkin = 0L;
        if (_wallSkinSymbol.valid())
        {
            if (_wallResLib.valid())
            {
                SkinSymbol querySymbol(*_wallSkinSymbol.get());
                querySymbol.objectHeight() = fabs(height);
                wallSkin = _wallResLib->getSkin(&querySymbol, _wallSkinPRNG, context.getDBOptions());
#ifdef _DEBUG
                if (!wallSkin)
                {
                    ++fubar;
                }
#endif
            }

            else
            {
                //TODO: simple single texture?
            }
        }


        // calculate the rooftop texture:
        SkinResource* roofSkin = 0L;
        if (!roofTextureIsFromImage)
        {
            if (_roofSkinSymbol.valid())
            {
                if (_roofResLib.valid())
                {
                    SkinSymbol querySymbol(*_roofSkinSymbol.get());
                    roofSkin = _roofResLib->getSkin(&querySymbol, _roofSkinPRNG, context.getDBOptions());
                }

                else
                {
                    //TODO: simple single texture?
                }
            }
        }

        float verticalOffset = (float)input->getDouble("__oe_verticalOffset", 0.0);

        // Build the data model for the structure.

        Structure structure;

        buildStructure(
            part,
            height,
            _extrusionSymbol->flatten().get(),
            verticalOffset,
            wallSkin,
            roofSkin,
            structure,
            context,
            roofTextureIsFromImage,
            roofTextureImageName,
            roofTextureHasMatIndex,
            roofTextureMatIndexName,
            RoofHelper
        );

        // Create the walls.
        if (walls.valid())
        {
            osg::Vec4f wallColor(1, 1, 1, 1), wallBaseColor(1, 1, 1, 1);

            if (_wallPolygonSymbol.valid())
            {
                wallColor = _wallPolygonSymbol->fill()->color();
            }

            if (_extrusionSymbol->wallGradientPercentage().isSet())
            {
                wallBaseColor = Color(wallColor).brightness(1.0 - *_extrusionSymbol->wallGradientPercentage());
            }
            else
            {
                wallBaseColor = wallColor;
            }

            buildWallGeometry(structure, walls.get(), wallColor, wallBaseColor, wallSkin);

            if (wallSkin)
            {
                // Get a stateset for the individual wall stateset
                context.resourceCache()->getOrCreateStateSet(wallSkin, wallStateSet, context.getDBOptions());
                if (hasHoles)
                    wallStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT_AND_BACK), osg::StateAttribute::ON);
                else
                {
                    //                      wallStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON);
                    wallStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);
                    wallStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
                }
                if (wallSkin->materialURI().isSet())
                {
                    context.resourceCache()->getOrCreateMatStateSet(wallSkin, wallStateSet, context.getDBOptions());
                    wallStateSet->setTextureMode(1, GL_TEXTURE_2D, osg::StateAttribute::ON);
                }

            }
        }

        // tessellate and add the roofs if necessary:
        if (rooflines.valid())
        {
            osg::Vec4f roofColor(1, 1, 1, 1);
            if (_roofPolygonSymbol.valid())
            {
                roofColor = _roofPolygonSymbol->fill()->color();
            }

            buildRoofGeometry(structure, rooflines.get(), roofColor, roofSkin, roofTextureIsFromImage, roofTextureHasMatIndex);
            if (roofTextureIsFromImage)
            {
                roofStateSet = RoofHelper->CreateSetState(context.getDBOptions());
                roofStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);
                roofStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
                if (roofTextureHasMatIndex)
                {
                    RoofHelper->AddMat2SetState(roofStateSet, context.getDBOptions());
                    roofStateSet->setTextureMode(1, GL_TEXTURE_2D, osg::StateAttribute::ON);
                }
            }
            else if (roofSkin)
            {
                // Get a stateset for the individual roof skin
                context.resourceCache()->getOrCreateStateSet(roofSkin, roofStateSet, context.getDBOptions());
                roofStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
                if (roofSkin->materialURI().isSet())
                {
                    context.resourceCache()->getOrCreateMatStateSet(roofSkin, roofStateSet, context.getDBOptions());
                    roofStateSet->setTextureMode(1, GL_TEXTURE_2D, osg::StateAttribute::ON);
                }
            }
        }

        if (outlines.valid())
        {
            osg::Vec4f outlineColor(1, 1, 1, 1);
            if (_outlineSymbol.valid())
            {
                outlineColor = _outlineSymbol->stroke()->color();
            }

            float minCreaseAngle = _outlineSymbol->creaseAngle().value();
            buildOutlineGeometry(structure, outlines.get(), outlineColor, minCreaseAngle);
        }

        if (baselines.valid())
        {
            //TODO.
            osgUtil::Tessellator tess;
            tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
            tess.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
            tess.retessellatePolygons(*(baselines.get()));
        }

        // Set up for feature naming and feature indexing:
        std::string name;
        if (!_featureNameExpr.empty())
            name = input->eval(_featureNameExpr, &context);

        FeatureIndexBuilder* index = context.featureIndex();

        if (walls.valid() && walls->getVertexArray() && walls->getVertexArray()->getNumElements() > 0)
        {
            addDrawable(walls.get(), wallStateSet.get(), name, input, index);
        }

        if (rooflines.valid() && rooflines->getVertexArray() && rooflines->getVertexArray()->getNumElements() > 0)
        {
            addDrawable(rooflines.get(), roofStateSet.get(), name, input, index);
        }

        if (baselines.valid() && baselines->getVertexArray() && baselines->getVertexArray()->getNumElements() > 0)
        {
            addDrawable(baselines.get(), 0L, name, input, index);
        }

        if (outlines.valid() && outlines->getVertexArray() && outlines->getVertexArray()->getNumElements() > 0)
        {
            addDrawable(outlines.get(), 0L, name, input, index);
        }
    }
    if (RoofHelper)
        delete RoofHelper;
    return true;
}

bool
ExtrudeGeometryFilter::ConstructAFeature(Feature* input, FilterContext& context)
{
#ifdef _DEBUG
    int fubar = 0;
#endif
    // run a symbol script if present.
    if (_extrusionSymbol->script().isSet())
    {
        StringExpression temp(_extrusionSymbol->script().get());
        input->eval(temp, &context);
    }

    bool roofTextureIsFromImage = false;
    std::string roofTextureImageName = "";
    bool roofTextureHasMatIndex = false;
    std::string roofTextureMatIndexName = "";
    ExtrudeGeomGDALHelper* RoofHelper = NULL;
    if (input->hasAttr("UseNameAsRoofTexture") && input->hasAttr("teximagename"))
    {
        if (input->getString("UseNameAsRoofTexture") == "true")
        {
            roofTextureImageName = input->getString("teximagename");
            roofTextureIsFromImage = true;
            if (input->hasAttr("MatIndexUrl"))
            {
                roofTextureHasMatIndex = true;
                roofTextureMatIndexName = input->getString("MatIndexUrl");
            }
        }
    }
    if (roofTextureIsFromImage)
    {
        RoofHelper = new ExtrudeGeomGDALHelper(roofTextureImageName);
        if (roofTextureHasMatIndex)
        {
            RoofHelper->Add_Mat_Index(roofTextureHasMatIndex, roofTextureMatIndexName);
        }
    }

    GeoStructure structure;

    SetLocalizationForFeature(context, input, structure);

    // iterator over the parts.
    bool hasHoles = false;

    Geometry* part = input->getGeometry();

    osg::ref_ptr<osg::Geometry> walls = new osg::Geometry();

    osg::ref_ptr<osg::Geometry> rooflines = 0L;
    osg::ref_ptr<osg::Geometry> baselines = 0L;
    osg::ref_ptr<osg::Geometry> outlines = 0L;
    rooflines = new osg::Geometry();


    // fire up the outline geometry if we have a line symbol.
    if (_outlineSymbol != 0L)
    {
        outlines = new osg::Geometry();
    }

    // make a base cap if we're doing stencil volumes.
    if (_makeStencilVolume)
    {
        baselines = new osg::Geometry();
    }

    // calculate the extrusion height:
    float height;

    if (_heightCallback.valid())
    {
        height = _heightCallback->operator()(input, context);
    }
    else if (_heightExpr.isSet())
    {
        height = input->eval(_heightExpr.mutable_value(), &context);
    }
    else
    {
        height = *_extrusionSymbol->height();
    }

    osg::ref_ptr<osg::StateSet> wallStateSet;
    osg::ref_ptr<osg::StateSet> roofStateSet;

    // calculate the wall texturing:
    SkinResource* wallSkin = 0L;
    if (_wallSkinSymbol.valid())
    {
        if (_wallResLib.valid())
        {
            SkinSymbol querySymbol(*_wallSkinSymbol.get());
            querySymbol.objectHeight() = fabs(height);
            wallSkin = _wallResLib->getSkin(&querySymbol, _wallSkinPRNG, context.getDBOptions());
#ifdef _DEBUG
            if (!wallSkin)
            {
                ++fubar;
            }
#endif
        }

        else
        {
            //TODO: simple single texture?
        }
    }


    // calculate the rooftop texture:
    SkinResource* roofSkin = 0L;
    if (!roofTextureIsFromImage)
    {
        if (_roofSkinSymbol.valid())
        {
            if (_roofResLib.valid())
            {
                SkinSymbol querySymbol(*_roofSkinSymbol.get());
                roofSkin = _roofResLib->getSkin(&querySymbol, _roofSkinPRNG, context.getDBOptions());
            }

            else
            {
                //TODO: simple single texture?
            }
        }
    }

    float verticalOffset = (float)input->getDouble("__oe_verticalOffset", 0.0);

    // Build the data model for the structure.

//  Structure roof_structure;
//  Structure wall_structure;

    LoadStructure(
        part,
        height,
        _extrusionSymbol->flatten().get(),
        verticalOffset,
        wallSkin,
        roofSkin,
        structure,
        context,
        roofTextureIsFromImage,
        roofTextureImageName,
        roofTextureHasMatIndex,
        roofTextureMatIndexName,
        RoofHelper
    );

    // Create the walls.
    if (walls.valid())
    {
        osg::Vec4f wallColor(1, 1, 1, 1), wallBaseColor(1, 1, 1, 1);

        if (_wallPolygonSymbol.valid())
        {
            wallColor = _wallPolygonSymbol->fill()->color();
        }

        if (_extrusionSymbol->wallGradientPercentage().isSet())
        {
            wallBaseColor = Color(wallColor).brightness(1.0 - *_extrusionSymbol->wallGradientPercentage());
        }
        else
        {
            wallBaseColor = wallColor;
        }

        FormWallGeometry(structure, walls.get(), wallColor, wallBaseColor, wallSkin);

        if (wallSkin)
        {
            // Get a stateset for the individual wall stateset
            context.resourceCache()->getOrCreateStateSet(wallSkin, wallStateSet, context.getDBOptions());
            if (hasHoles)
                wallStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT_AND_BACK), osg::StateAttribute::ON);
            else
            {
                //                      wallStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::BACK), osg::StateAttribute::ON);
                wallStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);
                wallStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
            }
            if (wallSkin->materialURI().isSet())
            {
                context.resourceCache()->getOrCreateMatStateSet(wallSkin, wallStateSet, context.getDBOptions());
                wallStateSet->setTextureMode(1, GL_TEXTURE_2D, osg::StateAttribute::ON);
            }

        }
    }

    // tessellate and add the roofs if necessary:
    if (rooflines.valid())
    {
        osg::Vec4f roofColor(1, 1, 1, 1);
        if (_roofPolygonSymbol.valid())
        {
            roofColor = _roofPolygonSymbol->fill()->color();
        }

        FormRoofGeometry(structure, rooflines.get(), roofColor, roofSkin, roofTextureIsFromImage, roofTextureHasMatIndex);

        if (roofTextureIsFromImage)
        {
            roofStateSet = RoofHelper->CreateSetState(context.getDBOptions());
            roofStateSet->setAttributeAndModes(new osg::CullFace(osg::CullFace::FRONT), osg::StateAttribute::ON);
            roofStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
            if (roofTextureHasMatIndex)
            {
                RoofHelper->AddMat2SetState(roofStateSet, context.getDBOptions());
                roofStateSet->setTextureMode(1, GL_TEXTURE_2D, osg::StateAttribute::ON);
            }
        }
        else if (roofSkin)
        {
            // Get a stateset for the individual roof skin
            context.resourceCache()->getOrCreateStateSet(roofSkin, roofStateSet, context.getDBOptions());
            roofStateSet->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON);
            if (roofSkin->materialURI().isSet())
            {
                context.resourceCache()->getOrCreateMatStateSet(roofSkin, roofStateSet, context.getDBOptions());
                roofStateSet->setTextureMode(1, GL_TEXTURE_2D, osg::StateAttribute::ON);
            }
        }
    }

    if (outlines.valid())
    {
        osg::Vec4f outlineColor(1, 1, 1, 1);
        if (_outlineSymbol.valid())
        {
            outlineColor = _outlineSymbol->stroke()->color();
        }

        float minCreaseAngle = _outlineSymbol->creaseAngle().value();
//      buildOutlineGeometry(structure, outlines.get(), outlineColor, minCreaseAngle);
    }

    if (baselines.valid())
    {
        //TODO.
        osgUtil::Tessellator tess;
        tess.setTessellationType(osgUtil::Tessellator::TESS_TYPE_GEOMETRY);
        tess.setWindingType(osgUtil::Tessellator::TESS_WINDING_ODD);
        tess.retessellatePolygons(*(baselines.get()));
    }

    // Set up for feature naming and feature indexing:
    std::string name;
    if (!_featureNameExpr.empty())
        name = input->eval(_featureNameExpr, &context);

    FeatureIndexBuilder* index = context.featureIndex();

    if (walls.valid() && walls->getVertexArray() && walls->getVertexArray()->getNumElements() > 0)
    {
        addDrawable(walls.get(), wallStateSet.get(), name, input, index);
    }

    if (rooflines.valid() && rooflines->getVertexArray() && rooflines->getVertexArray()->getNumElements() > 0)
    {
        addDrawable(rooflines.get(), roofStateSet.get(), name, input, index);
    }

    if (baselines.valid() && baselines->getVertexArray() && baselines->getVertexArray()->getNumElements() > 0)
    {
        addDrawable(baselines.get(), 0L, name, input, index);
    }

    if (outlines.valid() && outlines->getVertexArray() && outlines->getVertexArray()->getNumElements() > 0)
    {
        addDrawable(outlines.get(), 0L, name, input, index);
    }
    if (RoofHelper)
        delete RoofHelper;
    return true;
}

void ExtrudeGeometryFilter::InitForTile(FilterContext& context)
{
    reset(context);

    // minimally, we require an extrusion symbol.
    if (!_extrusionSymbol.valid())
    {
        OE_WARN << LC << "Missing required extrusion symbolology; geometry will be empty" << std::endl;
        return;
    }

    // establish the active resource library, if applicable.
    _wallResLib = 0L;
    _roofResLib = 0L;

    const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

    if (sheet != 0L)
    {
        if (_wallSkinSymbol.valid() && _wallSkinSymbol->library().isSet())
        {
            _wallResLib = sheet->getResourceLibrary(*_wallSkinSymbol->library());

            if (!_wallResLib.valid())
            {
                OE_WARN << LC << "Unable to load resource library '" << *_wallSkinSymbol->libraryName() << "'"
                    << "; wall geometry will not be textured." << std::endl;
                _wallSkinSymbol = 0L;
            }
        }

        if (_roofSkinSymbol.valid() && _roofSkinSymbol->library().isSet())
        {
            _roofResLib = sheet->getResourceLibrary(*_roofSkinSymbol->library());
            if (!_roofResLib.valid())
            {
                OE_WARN << LC << "Unable to load resource library '" << *_roofSkinSymbol->library() << "'"
                    << "; roof geometry will not be textured." << std::endl;
                _roofSkinSymbol = 0L;
            }
        }
    }

    // calculate the localization matrices (_local2world and _world2local)
    computeLocalizers(context);

    // seed our random number generators
    if (_wallSkinSymbol.valid())
        _wallSkinPRNG.seed(*_wallSkinSymbol->randomSeed());
    else
        _wallSkinPRNG.seed(0);
    if (_roofSkinSymbol.valid())
        _roofSkinPRNG.seed(*_roofSkinSymbol->randomSeed());
    else
        _roofSkinPRNG.seed(0);

}


osg::Node * ExtrudeGeometryFilter::ExtrudeFeature(Feature* input, FilterContext& context)
{
    reset(context);
    // calculate the localization matrices (_local2world and _world2local)
    Geometry * geometry = input->getGeometry();
    ConstGeometryIterator zfinder(geometry);
    int num = 0;
    osg::Vec3d centroid(0.0, 0.0, 0.0);
    while (zfinder.hasMore())
    {
        const Geometry* geom = zfinder.next();
        for (Geometry::const_iterator m = geom->begin(); m != geom->end(); ++m)
        {
            osg::Vec3d point = *m;
            centroid.x() += point.x();
            centroid.y() += point.y();
            ++num;
        }
    }
    if (num > 0)
    {
        centroid.x() /= (double)num;
        centroid.y() /= (double)num;
    }

    computeLocalizers(centroid, context);

    ExtrudeAFeature(input, context);

    // parent geometry with a delocalizer (if necessary)
    osg::Group* group = createDelocalizeGroup();

    // add all the geodes
    for (SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i)
    {
        group->addChild(i->second.get());
    }
    _geodes.clear();

    if (_mergeGeometry == true && _featureNameExpr.empty())
    {
        osgUtil::Optimizer::MergeGeometryVisitor mg;
        mg.setTargetMaximumNumberOfVertices(65536);
        group->accept(mg);

        // Because the mesh optimizers damaga line geometry.
        if (!_outlineSymbol.valid())
        {
            osgUtil::Optimizer o;
            o.optimize(group,
                osgUtil::Optimizer::INDEX_MESH |
                osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                osgUtil::Optimizer::VERTEX_POSTTRANSFORM);
        }
    }

    // Prepare buffer objects.
    AllocateAndMergeBufferObjectsVisitor allocAndMerge;
    group->accept(allocAndMerge);

    // set a uniform indicating that clamping attributes are available.
    Clamping::installHasAttrsUniform(group->getOrCreateStateSet());

    // if we drew outlines, apply a poly offset too.
    if (_outlineSymbol.valid())
    {
        osg::StateSet* groupStateSet = group->getOrCreateStateSet();
        groupStateSet->setAttributeAndModes(new osg::PolygonOffset(1, 1), 1);
        if (_outlineSymbol->stroke()->width().isSet())
            groupStateSet->setAttributeAndModes(new osg::LineWidth(*_outlineSymbol->stroke()->width()), 1);
    }

    return group;

}

osg::Node* ExtrudeGeometryFilter::ConstructFeature(Feature* input, FilterContext& context)
{
    reset(context);
    // calculate the localization matrices (_local2world and _world2local)
    Geometry* geometry = input->getGeometry();
    ConstGeometryIterator zfinder(geometry);
    int num = 0;
    osg::Vec3d centroid(0.0, 0.0, 0.0);
    while (zfinder.hasMore())
    {
        const Geometry* geom = zfinder.next();
        for (Geometry::const_iterator m = geom->begin(); m != geom->end(); ++m)
        {
            osg::Vec3d point = *m;
            centroid.x() += point.x();
            centroid.y() += point.y();
            ++num;
        }
    }
    if (num > 0)
    {
        centroid.x() /= (double)num;
        centroid.y() /= (double)num;
    }

    computeLocalizers(centroid, context);

    ConstructAFeature(input, context);

    // parent geometry with a delocalizer (if necessary)
    osg::Group* group = createDelocalizeGroup();

    // add all the geodes
    for (SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i)
    {
        group->addChild(i->second.get());
    }
    _geodes.clear();

    if (_mergeGeometry == true && _featureNameExpr.empty())
    {
        osgUtil::Optimizer::MergeGeometryVisitor mg;
        mg.setTargetMaximumNumberOfVertices(65536);
        group->accept(mg);

        // Because the mesh optimizers damaga line geometry.
        if (!_outlineSymbol.valid())
        {
            osgUtil::Optimizer o;
            o.optimize(group,
                osgUtil::Optimizer::INDEX_MESH |
                osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                osgUtil::Optimizer::VERTEX_POSTTRANSFORM);
        }
    }

    // Prepare buffer objects.
    AllocateAndMergeBufferObjectsVisitor allocAndMerge;
    group->accept(allocAndMerge);

    // set a uniform indicating that clamping attributes are available.
    Clamping::installHasAttrsUniform(group->getOrCreateStateSet());

    // if we drew outlines, apply a poly offset too.
    if (_outlineSymbol.valid())
    {
        osg::StateSet* groupStateSet = group->getOrCreateStateSet();
        groupStateSet->setAttributeAndModes(new osg::PolygonOffset(1, 1), 1);
        if (_outlineSymbol->stroke()->width().isSet())
            groupStateSet->setAttributeAndModes(new osg::LineWidth(*_outlineSymbol->stroke()->width()), 1);
    }

    return group;

}

osg::Node*
ExtrudeGeometryFilter::push( FeatureList& input, FilterContext& context )
{
    reset( context );

    // minimally, we require an extrusion symbol.
    if ( !_extrusionSymbol.valid() )
    {
        OE_WARN << LC << "Missing required extrusion symbolology; geometry will be empty" << std::endl;
        return new osg::Group();
    }

    // establish the active resource library, if applicable.
    _wallResLib = 0L;
    _roofResLib = 0L;

    const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

    if ( sheet != 0L )
    {
        if ( _wallSkinSymbol.valid() && _wallSkinSymbol->library().isSet() )
        {
            _wallResLib = sheet->getResourceLibrary( *_wallSkinSymbol->library() );

            if ( !_wallResLib.valid() )
            {
                OE_WARN << LC << "Unable to load resource library '" << *_wallSkinSymbol->libraryName() << "'"
                    << "; wall geometry will not be textured." << std::endl;
                _wallSkinSymbol = 0L;
            }
        }

        if ( _roofSkinSymbol.valid() && _roofSkinSymbol->library().isSet() )
        {
            _roofResLib = sheet->getResourceLibrary( *_roofSkinSymbol->library() );
            if ( !_roofResLib.valid() )
            {
                OE_WARN << LC << "Unable to load resource library '" << *_roofSkinSymbol->library() << "'"
                    << "; roof geometry will not be textured." << std::endl;
                _roofSkinSymbol = 0L;
            }
        }
    }

    // calculate the localization matrices (_local2world and _world2local)
    computeLocalizers( context );

    // push all the features through the extruder.
    bool ok = process( input, context );

    // parent geometry with a delocalizer (if necessary)
    osg::Group* group = createDelocalizeGroup();
    
    // add all the geodes
    for( SortedGeodeMap::iterator i = _geodes.begin(); i != _geodes.end(); ++i )
    {
        group->addChild( i->second.get() );
    }
    _geodes.clear();

    if ( _mergeGeometry == true && _featureNameExpr.empty() )
    {
        osgUtil::Optimizer::MergeGeometryVisitor mg;
        mg.setTargetMaximumNumberOfVertices(65536);
        group->accept(mg);

        // Because the mesh optimizers damaga line geometry.
        if ( !_outlineSymbol.valid() )
        {
            osgUtil::Optimizer o;
            o.optimize(group,
                osgUtil::Optimizer::INDEX_MESH |
                osgUtil::Optimizer::VERTEX_PRETRANSFORM |
                osgUtil::Optimizer::VERTEX_POSTTRANSFORM );
        }
    }

    // Prepare buffer objects.
    AllocateAndMergeBufferObjectsVisitor allocAndMerge;
    group->accept( allocAndMerge );

    // set a uniform indicating that clamping attributes are available.
    Clamping::installHasAttrsUniform( group->getOrCreateStateSet() );

    // if we drew outlines, apply a poly offset too.
    if ( _outlineSymbol.valid() )
    {
        osg::StateSet* groupStateSet = group->getOrCreateStateSet();
        groupStateSet->setAttributeAndModes( new osg::PolygonOffset(1,1), 1 );
        if ( _outlineSymbol->stroke()->width().isSet() )
            groupStateSet->setAttributeAndModes( new osg::LineWidth(*_outlineSymbol->stroke()->width()), 1 );
    }

    return group;
}

ExtrudeGeomGDALHelper::ExtrudeGeomGDALHelper(std::string gdalimagename) : _gdalimagename(gdalimagename), _have_image_data(false), _num_bands(0), _RsizeX(0), _RsizeY(0),
																		 _minX(0.0), _maxY(0.0), _maxX(0.0), _minY(0.0), _imagenamefortexture(""), _have_mat_index_data(false),
																		 _mat_index_name("")
{
	Open_Image();
}

ExtrudeGeomGDALHelper::~ExtrudeGeomGDALHelper()
{

}

void ExtrudeGeomGDALHelper::Open_Image()
{
	GDALDataset *poDataset;
	poDataset = (GDALDataset *)GDALOpen(_gdalimagename.c_str(), GA_ReadOnly);
	if (poDataset == NULL)
		return;

	OGRSpatialReference  *hSRS = new OGRSpatialReference();
	char		      *pszProjection;

	pszProjection = (char *)GDALGetProjectionRef(poDataset);
	if (pszProjection == NULL)
	{
		GDALClose(poDataset);
		return;
	}

	hSRS->importFromWkt(&pszProjection);
	if (!hSRS->IsGeographic())
	{
		delete hSRS;
		GDALClose(poDataset);
	}
    hSRS->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

	_num_bands = poDataset->GetRasterCount();
	_RsizeX = poDataset->GetRasterXSize();
	_RsizeY = poDataset->GetRasterYSize();

	GDALGetGeoTransform(poDataset, _adfGeoTransform);
	//Could add transform checks here
	_minX = _adfGeoTransform[GEOTRSFRM_TOPLEFT_X];
	_maxY = _adfGeoTransform[GEOTRSFRM_TOPLEFT_Y];
	_maxX = _minX + (double)_RsizeX * _adfGeoTransform[GEOTRSFRM_WE_RES];
	_minY = _maxY + (double)_RsizeY * _adfGeoTransform[GEOTRSFRM_NS_RES];

	int pos = _gdalimagename.find(".tif");
	if (pos != std::string::npos)
	{
		_imagenamefortexture = _gdalimagename.substr(0, pos);
		_imagenamefortexture += ".rgb";
	}

	GDALClose(poDataset);
	delete hSRS;

	_have_image_data = true;

	return;
}

bool ExtrudeGeomGDALHelper::ImageOK(void)
{
	return _have_image_data;
}

bool ExtrudeGeomGDALHelper::LL2UV(osg::Vec3d Point, float &u, float &v)
{
	if (_have_image_data)
	{
		double xRel = Point.x() - _minX;
		double yRel = _maxY - Point.y();
		float Pixx = xRel / _adfGeoTransform[GEOTRSFRM_WE_RES];
		float Pixy = yRel / abs(_adfGeoTransform[GEOTRSFRM_NS_RES]);
		u = Pixx / (float)_RsizeX;
		v = 1.0 - (Pixy / (float)_RsizeY);
		return true;
	}
	else
		return false;
}

void ExtrudeGeomGDALHelper::Add_Mat_Index(bool have_mat_index, std::string mat_index_name)
{
	_have_mat_index_data = have_mat_index;
	_mat_index_name = mat_index_name;
}

osg::StateSet * ExtrudeGeomGDALHelper::CreateSetState(const osgDB::Options* readOptions)
{

	osg::StateSet * TextureState = NULL;
	osg::ref_ptr<osg::Image> image = osgDB::readImageFile(_imagenamefortexture, readOptions);
	if (image)
	{
		TextureState = new osg::StateSet();
		osg::Texture * tex = CreateTexture(image);
		if (tex)
		{
			TextureState->setTextureAttributeAndModes(0, tex, osg::StateAttribute::ON);
		}
	}
	return TextureState;
}

osg::Texture * ExtrudeGeomGDALHelper::CreateTexture(osg::Image *image)
{
	if (!image) return 0L;

	osg::Texture* tex;

	tex = new osg::Texture2D(image);
	tex->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
	tex->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);

	tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::LINEAR_MIPMAP_LINEAR);
	tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::LINEAR);

	// skin textures are likely to be shared, paged, etc. so keep them in memory.
	tex->setUnRefImageDataAfterApply(false);

	// don't resize them, let it be
	tex->setResizeNonPowerOfTwoHint(false);

	return tex;

}

bool ExtrudeGeomGDALHelper::AddMat2SetState(osg::ref_ptr<osg::StateSet> &roofSetState, const osgDB::Options* readOptions)
{
	bool valid = false;
	if (_have_mat_index_data)
	{
		osg::ref_ptr<osg::Image> image = osgDB::readImageFile(_mat_index_name, readOptions);
		if (image)
		{
			osg::Texture * tex = CreateTexture(image);
			if (tex)
			{
				int ntx = roofSetState->getNumTextureModeLists();
				roofSetState->setTextureAttributeAndModes(ntx, tex, osg::StateAttribute::ON);
				valid = true;
			}
		}
	}
	return valid;
}