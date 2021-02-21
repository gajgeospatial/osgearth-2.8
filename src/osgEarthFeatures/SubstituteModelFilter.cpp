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
#include <osgEarthFeatures/SubstituteModelFilter>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/Session>
#include <osgEarthFeatures/GeometryUtils>
#include <osgEarthSymbology/MeshConsolidator>
#include <osgEarthSymbology/MeshFlattener>
#include <osgEarth/ECEF>
#include <osgEarth/VirtualProgram>
#include <osgEarth/DrawInstanced>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/CullingUtils>
#include <ogc/ogc_IE>
#include <CDB_TileLib/CDB_Tile>
#include <osg/AutoTransform>
#include <osg/Drawable>
#include <osg/Geode>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>
#include <osg/AlphaFunc>
#include <osg/Billboard>
#include <osg/CullFace>

#include <osgSim/LightPointNode>

#include <osgDB/FileNameUtils>
#include <osgDB/Registry>
#include <osgDB/WriteFile>
#include <osgDB/Archive>

#include <osgUtil/Optimizer>
#include <osgUtil/MeshOptimizers>

#include <list>
#include <deque>

#define LC "[SubstituteModelFilter] "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

//------------------------------------------------------------------------

namespace
{
    static osg::Node* s_defaultModel =0L;

    struct SetSmallFeatureCulling : public osg::NodeCallback
    {
        bool _value;
        SetSmallFeatureCulling(bool value) : _value(value) { }
        void operator()(osg::Node* node, osg::NodeVisitor* nv) {
            Culling::asCullVisitor(nv)->setSmallFeatureCullingPixelSize(-1.0f);
            traverse(node, nv);
        }
    };
}


class MultipleTextureVisitor : public osg::NodeVisitor {
public:
	MultipleTextureVisitor()
		: osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
	{
	}

	virtual void apply(osg::Geode& geode)
	{
		apply(geode.getStateSet());
		for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
		{
			osg::Drawable* drawable = geode.getDrawable(i);
			apply(drawable->getStateSet());
		}
		osg::NodeVisitor::apply(geode);
	}

	void apply(osg::StateSet* ss)
	{

		if (ss)
		{
#ifdef _DEBUG
			int numTexModes = ss->getNumTextureModeLists();
			if(numTexModes > 0)
				ss->setTextureMode(0, GL_TEXTURE_2D, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
#endif

#if 0
			osg::CullFace * sa = static_cast<osg::CullFace *>(ss->getAttribute(osg::StateAttribute::CULLFACE));
			if (sa)
			{
				ss->removeAttribute(sa);
			}
#endif
			for (unsigned int i = 1; i<ss->getNumTextureModeLists(); i++)
			{
				const osg::Texture* texture = dynamic_cast<osg::Texture*>(ss->getTextureAttribute(i, osg::StateAttribute::TEXTURE));
				if (texture)
				{
					ss->setTextureMode(i, GL_TEXTURE_2D, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
				}
			}
		}
	}
};


//------------------------------------------------------------------------

SubstituteModelFilter::SubstituteModelFilter( const Style& style ) :
_style                ( style ),
_cluster              ( false ),
_useDrawInstanced     ( false ),
_merge                ( true ),
_normalScalingRequired( false ),
_instanceCache        ( false ),    // cache per object so MT not required
_GlobalModelMgr	      (GlobalModelManager::getInstance())
{
    //NOP
}

bool
SubstituteModelFilter::findResource(const URI&            uri,
                                    const InstanceSymbol* symbol,
                                    FilterContext&        context, 
                                    std::set<URI>&        missing,
                                    osg::ref_ptr<InstanceResource>& output )
{
    // be careful about refptrs here since _instanceCache is an LRU.

    InstanceCache::Record rec;
    if ( _instanceCache.get(uri, rec) )
    {
        // found it in the cache:
        output = rec.value().get();
    }
    else if ( _resourceLib.valid() )
    {
        // look it up in the resource library:
        output = _resourceLib->getInstance( uri.base(), context.getDBOptions() );
    }
    else
    {
        // create it on the fly:
        output = symbol->createResource();
        output->uri() = uri;
        _instanceCache.insert( uri, output.get() );
    }

    // failed to find the instance.
    if ( !output.valid() )
    {
        if ( missing.find(uri) == missing.end() )
        {
            missing.insert(uri);
            OE_WARN << LC << "Failed to locate resource: " << uri.full() << std::endl;
        }
    }

    return output.valid();
}

bool
SubstituteModelFilter::process(const FeatureList&           features,
                               const InstanceSymbol*        symbol,
                               Session*                     session,
                               osg::Group*                  attachPoint,
                               FilterContext&               context )
{
    // Establish SRS information:
    bool makeECEF = context.getSession()->getMapInfo().isGeocentric();
    const SpatialReference* targetSRS = context.getSession()->getMapInfo().getSRS();

    // first, go through the features and build the model cache. Apply the model matrix' scale
    // factor to any AutoTransforms directly (cloning them as necessary)
    std::map< std::pair<URI, float>, osg::ref_ptr<osg::Node> > uniqueModels;

    // URI cache speeds up URI creation since it can be slow.
    osgEarth::fast_map<std::string, URI> uriCache;

    // keep track of failed URIs so we don't waste time or warning messages on them
    std::set< URI > missing;

	//CDB Model Replacement Data
	ModelReplacmentdataPV	ReplaceModels;

	StringExpression  uriEx    = *symbol->url();
    NumericExpression scaleEx  = *symbol->scale();

    const ModelSymbol* modelSymbol = dynamic_cast<const ModelSymbol*>(symbol);
    const IconSymbol*  iconSymbol  = dynamic_cast<const IconSymbol*> (symbol);

    NumericExpression headingEx;    
    NumericExpression scaleXEx;
    NumericExpression scaleYEx;
    NumericExpression scaleZEx;

    if ( modelSymbol )
    {
        headingEx = *modelSymbol->heading();
        scaleXEx  = *modelSymbol->scaleX();
        scaleYEx  = *modelSymbol->scaleY();
        scaleZEx  = *modelSymbol->scaleZ();
    }
	osg::ref_ptr<osgDB::Archive> ar = NULL;

	bool Do_Editing_Support = false;

	MultipleTextureVisitor v;

#ifdef _DO_GPKG_TESTS
	OGC_IE_Tracking * tracker = OGC_IE_Tracking::getInstance();
	bool doTracking = (tracker->Get_Test() != NO_IE_Test);
	bool trackingSet = false;
	CDB_Tile_Type tt = CDB_Unknown;

#endif

    for( FeatureList::const_iterator f = features.begin(); f != features.end(); ++f )
    {
        Feature* input = f->get();
		std::string archiveName;
        // Run a feature pre-processing script.
        if ( symbol->script().isSet() )
        {
            StringExpression scriptExpr(symbol->script().get());
            input->eval( scriptExpr, &context );
        }
		if ((ar == NULL) && input->hasAttr("osge_modelzip"))  
		{
			//all model requests should come from the same archive
			//the existance and validity of the archive has already been tested by the driver
			archiveName = input->getString("osge_modelzip");
			ar = osgDB::openArchive(archiveName, osgDB::ReaderWriter::ArchiveStatus::READ);

		}
#ifdef _DO_GPKG_TESTS
		if (!trackingSet)
		{
			if (doTracking)
			{
				if (ar)
					tt = GeoSpecificModel;
				else
					tt = GeoTypicalModel;
				tracker->StartTileLoad(tt);
			}
			trackingSet = true;
		}
#endif
		bool feature_defined_model = input->hasAttr("osge_modelname");
		bool skip_multitexdisable = input->hasAttr("osge_nomultidisable");
		bool replace = false;

#ifdef _DEBUG
		int fubar = 0;
#endif
		std::string st;
		std::string modeltextPath;
		std::string referenceName;
		osg::ref_ptr<osgDB::Options> localoptions = NULL;
		// evaluate the instance URI expression:
		bool feature_defined_preInstanced = false;
		if (feature_defined_model)
		{
			st = input->getString("osge_modelname");
			replace = input->hasAttr("osge_referencedName");
			if (replace)
				referenceName = input->getString("osge_referencedName");
#ifdef _DEBUG
			if (replace)
			{
				++fubar;
			}
#endif
			if (input->hasAttr("osge_texturezip"))
			{
				localoptions = context.getSession()->getDBOptions()->cloneOptions();
				modeltextPath = input->getString("osge_texturezip");
				localoptions->setDatabasePath(modeltextPath);
				std::string options_string = localoptions->getOptionString();
				if (options_string.empty())
					options_string = "TextureInArchive";
				else
					options_string.append(";TextureInArchive");
				localoptions->setOptionString(options_string);
				if (input->hasAttr("osge_gs_uses_gt"))
				{
					std::string archiveRefPath = input->getString("osge_gs_uses_gt");
					osgDB::FilePathList& datapathlist = localoptions->getDatabasePathList();
					datapathlist.push_back(archiveRefPath);
				}
			}
			else if (input->hasAttr("osge_modeltexture"))
			{
				localoptions = context.getSession()->getDBOptions()->cloneOptions();
				modeltextPath = input->getString("osge_modeltexture");
				localoptions->setDatabasePath(modeltextPath);
				osgDB::FilePathList& datapathlist = localoptions->getDatabasePathList();
				datapathlist.push_back(st);
				std::string options_string = localoptions->getOptionString();
				if (options_string.empty())
					options_string = "Remap2Directory";
				else
					options_string.append(";Remap2Directory");
				localoptions->setOptionString(options_string);
				//add below up here??
				if (input->hasAttr("osge_gs_uses_gt"))
				{
					std::string archiveRefPath = input->getString("osge_gs_uses_gt");
					datapathlist.push_back(archiveRefPath);					
				}
			}
			else if (input->hasAttr("osge_gs_uses_gt"))
			{
				localoptions = context.getSession()->getDBOptions()->cloneOptions();
				std::string archiveRefPath = input->getString("osge_gs_uses_gt");
				localoptions->setDatabasePath(archiveRefPath);
			}
			else
				feature_defined_preInstanced = true;
		}
		else
		{
			st = input->eval(uriEx, &context);
		}

		URI& instanceURI = uriCache[st];
		if(instanceURI.empty()) // Create a map, to reuse URI's, since they take a long time to create
		{
			instanceURI = URI( st, uriEx.uriContext() );
		}

        // find the corresponding marker in the cache
        osg::ref_ptr<InstanceResource> instance;
        if ( !findResource(instanceURI, symbol, context, missing, instance) )
            continue;

        // evalute the scale expression (if there is one)
        float scale = 1.0f;
        osg::Vec3d scaleVec(1.0, 1.0, 1.0);
        osg::Matrixd scaleMatrix;
		if (feature_defined_model)
		{
			//add check for attribute scale
			float scaleX = 1.0; 
			float scaleY = 1.0;
			float scaleZ = 1.0;
			if (input->hasAttr("scalx"))
				scaleX = (float)input->getDouble("scalx");
			if (input->hasAttr("scaly"))
				scaleY = (float)input->getDouble("scaly");
			if (input->hasAttr("scalz"))
				scaleZ = (float)input->getDouble("scalz");
			if ((scaleX != 1.0) || (scaleY != 1.0) || (scaleZ != 1.0))
				_normalScalingRequired = true;
			scaleMatrix = osg::Matrix::scale(scaleX, scaleY, scaleZ);
			}
		else 
		{
 
        	if ( symbol->scale().isSet() )
        	{
           	 	scale = input->eval( scaleEx, &context );
            	scaleVec.set(scale, scale, scale);
        	}
        	if ( modelSymbol )
        	{
            	if ( modelSymbol->scaleX().isSet() )
            	{
                	scaleVec.x() *= input->eval( scaleXEx, &context );
            	}
            	if ( modelSymbol->scaleY().isSet() )
            	{
                	scaleVec.y() *= input->eval( scaleYEx, &context );
            	}
            	if ( modelSymbol->scaleZ().isSet() )
            	{
                	scaleVec.z() *= input->eval( scaleZEx, &context );
            	}
        	}

       	 	if ( scaleVec.x() == 0.0 ) scaleVec.x() = 1.0;
        	if ( scaleVec.y() == 0.0 ) scaleVec.y() = 1.0;
        	if ( scaleVec.z() == 0.0 ) scaleVec.z() = 1.0;

        	scaleMatrix = osg::Matrix::scale( scaleVec );
		}

        osg::Matrixd rotationMatrix;
		if (feature_defined_model)
		{
			if (input->hasAttr("ao1"))
			{
				float heading = (float)input->getDouble("ao1") * -1.0;
				if (heading != 0.0)
					rotationMatrix.makeRotate(osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0, 0, 1)));
			}
		}
		else
		{
			if (modelSymbol && modelSymbol->heading().isSet())
			{
				float heading = input->eval(headingEx, &context);
				rotationMatrix.makeRotate(osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0, 0, 1)));
			}
		}

		// how that we have a marker source, create a node for it
		std::pair<URI,float> key( instanceURI, iconSymbol? scale : 1.0f ); //use 1.0 for models, since we don't want unique models based on scaling

        // cache nodes per instance.
        osg::ref_ptr<osg::Node>& model = uniqueModels[key];
        if ( !model.valid() )
        {
            // Always clone the cached instance so we're not processing data that's
            // already in the scene graph. -gw
			if (feature_defined_model)
				context.resourceCache()->cloneOrCreateInstanceNode(instance.get(), model, localoptions, ar);
			else
            	context.resourceCache()->cloneOrCreateInstanceNode(instance.get(), model, context.getDBOptions());	
			if (model)
			{
				model->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
				if(!skip_multitexdisable)
					model->accept(v);
			}
            // if icon decluttering is off, install an AutoTransform.
            if ( iconSymbol )
            {
                if ( iconSymbol->declutter() == true )
                {
                    ScreenSpaceLayout::activate(model->getOrCreateStateSet());
                }
                else if ( dynamic_cast<osg::AutoTransform*>(model.get()) == 0L )
                {
                    osg::AutoTransform* at = new osg::AutoTransform();
                    at->setAutoRotateMode( osg::AutoTransform::ROTATE_TO_SCREEN );
                    at->setAutoScaleToScreen( true );
                    at->addChild( model );
                    model = at;
                }
            }
			if (model.valid() && feature_defined_model)
				model->setName(st.c_str());
        }

		if (replace && model.valid())
		{
			std::string newXformName = "";
			if (input->hasAttr("transformname"))
				newXformName = input->getString("transformname");
			ModelReplacementdataP ReplaceEntry = new ModelReplacementData(st, referenceName, newXformName, model);
			ReplaceModels.push_back(ReplaceEntry);
		}

        if ( model.valid() )
        {
            GeometryIterator gi( input->getGeometry(), false );
            while( gi.hasMore() )
            {
                Geometry* geom = gi.next();

                // if necessary, transform the points to the target SRS:
                if ( !makeECEF && !targetSRS->isEquivalentTo(context.profile()->getSRS()) )
                {
                    context.profile()->getSRS()->transform( geom->asVector(), targetSRS );
                }

                for( unsigned i=0; i<geom->size(); ++i )
                {
                    osg::Matrixd mat;

                    // need to recalcluate expression-based data per-point, not just per-feature!
                    float scale = 1.0f;
                    osg::Vec3d scaleVec(1.0, 1.0, 1.0);
					if (!feature_defined_model)
					{
						if (symbol->scale().isSet())
						{
							scale = input->eval(scaleEx, &context);
							scaleVec.set(scale, scale, scale);
						}
						if (modelSymbol)
						{
							if (modelSymbol->scaleX().isSet())
							{
								scaleVec.x() *= input->eval(scaleXEx, &context);
							}
							if (modelSymbol->scaleY().isSet())
							{
								scaleVec.y() *= input->eval(scaleYEx, &context);
							}
							if (modelSymbol->scaleZ().isSet())
							{
								scaleVec.z() *= input->eval(scaleZEx, &context);
							}
						}

						if (scaleVec.x() == 0.0) scaleVec.x() = 1.0;
						if (scaleVec.y() == 0.0) scaleVec.y() = 1.0;
						if (scaleVec.z() == 0.0) scaleVec.z() = 1.0;

						scaleMatrix = osg::Matrix::scale(scaleVec);
					}

                    if ( modelSymbol->heading().isSet() )
                    {
                        float heading = input->eval(headingEx, &context);
                        rotationMatrix.makeRotate( osg::Quat(osg::DegreesToRadians(heading), osg::Vec3(0,0,1)) );
                    }

                    osg::Vec3d point = (*geom)[i];
                    if ( makeECEF )
                    {
                        // the "rotation" element lets us re-orient the instance to ensure it's pointing up. We
                        // could take a shortcut and just use the current extent's local2world matrix for this,
                        // but if the tile is big enough the up vectors won't be quite right.
                        osg::Matrixd rotation;
                        ECEF::transformAndGetRotationMatrix( point, context.profile()->getSRS(), point, targetSRS, rotation );
                        mat = rotationMatrix * scaleMatrix * rotation * osg::Matrixd::translate( point ) * _world2local;
                    }
                    else
                    {
                        mat = rotationMatrix * scaleMatrix *  osg::Matrixd::translate( point ) * _world2local;
                    }

                    osg::MatrixTransform* xform = new osg::MatrixTransform();
                    xform->setMatrix( mat );
                    xform->setDataVariance( osg::Object::STATIC );
                    xform->addChild( model.get() );
                    attachPoint->addChild( xform );

                    // Only tag nodes if we aren't using clustering.
                    if ( context.featureIndex() && !_cluster)
                    {
                        context.featureIndex()->tagNode( xform, input );
                    }

                    // name the feature if necessary
					if (feature_defined_model)
					{
						if (input->hasAttr("transformname"))
						{
							Do_Editing_Support = true;
							std::string transname = input->getString("transformname");
							xform->setName(transname);
						}
					}
					else if ( !_featureNameExpr.empty() )
                    {
                        const std::string& name = input->eval( _featureNameExpr, &context);
                        if ( !name.empty() )
                            xform->setName( name );
                    }
                }
            }
        }
    }


	if (ar)
	{
		ar.release();
		ar = NULL;
	}


	if (Do_Editing_Support)
	{
		osg::Group * Orphaned = new osg::Group();
		Orphaned->setName("orphaneddmat");
		osg::MatrixTransform* world2local = new osg::MatrixTransform();
		world2local->setName("world2local");
		world2local->setMatrix(_world2local);
		Orphaned->addChild(world2local);
		attachPoint->addChild(Orphaned);
	}

#ifdef _DO_GPKG_TESTS
	if (trackingSet)
	{
		if (doTracking)
		{
			tracker->EndTileLoad(tt, features.size());
		}
	}
#endif

    if ( iconSymbol )
    {
        // activate decluttering for icons if requested
        if ( iconSymbol->declutter() == true )
        {
            ScreenSpaceLayout::activate(attachPoint->getOrCreateStateSet());
        }

        // activate horizon culling if we are in geocentric space
        if ( context.getSession() && context.getSession()->getMapInfo().isGeocentric() )
        {
            //TODO: re-evaluate this; use Horizon?
            HorizonCullingProgram::install( attachPoint->getOrCreateStateSet() );
        }
    }

    // active DrawInstanced if required:
    if ( _useDrawInstanced )
    {
        DrawInstanced::convertGraphToUseDrawInstanced( attachPoint );

        // install a shader program to render draw-instanced.
        DrawInstanced::install( attachPoint->getOrCreateStateSet() );
    }

	if (ReplaceModels.size() > 0)
		_GlobalModelMgr->Add_To_Replacment_Stack(ReplaceModels);

    return true;
}




namespace
{
    /**
     * Extracts unclusterable things like lightpoints and billboards from the given scene graph and copies them into a cloned scene graph
     * This actually just removes all geodes from the scene graph, so this could be applied to any other type of node that you want to keep
     * The geodes will be clustered together in the flattened graph.
     */
    osg::Node* extractUnclusterables(osg::Node* node)
    {
        // Clone the scene graph
        osg::ref_ptr< osg::Node > clone = (osg::Node*)node->clone(osg::CopyOp::DEEP_COPY_NODES);
       
        // Now remove any geodes
        FindNodesVisitor<osg::Geode> findGeodes;
        clone->accept(findGeodes);
        for (unsigned int i = 0; i < findGeodes._results.size(); i++)
        {
            osg::ref_ptr< osg::Geode > geode = findGeodes._results[i];
            

            // Special check for billboards.  Me want to keep them in this special graph of 
            // unclusterable stuff.
            osg::Billboard* billboard = dynamic_cast< osg::Billboard* >( geode.get() );
            

            if (geode->getNumParents() > 0 && !billboard)
            {
                // Get all the parents for the geode and remove it from them.
                std::vector< osg::ref_ptr< osg::Group > > parents;
                for (unsigned int j = 0; j < geode->getNumParents(); j++)
                {
                    parents.push_back(geode->getParent(j));
                }

                for (unsigned int j = 0; j < parents.size(); j++)
                {
                    parents[j]->removeChild(geode);
                }
            }
            
        }

        return clone.release();
    };
}

osg::Node*
SubstituteModelFilter::push(FeatureList& features, FilterContext& context)
{
    if ( !isSupported() )
    {
        OE_WARN << "SubstituteModelFilter support not enabled" << std::endl;
        return 0L;
    }

    if ( _style.empty() )
    {
        OE_WARN << LC << "Empty style; cannot process features" << std::endl;
        return 0L;
    }

    osg::ref_ptr<const InstanceSymbol> symbol = _style.get<InstanceSymbol>();

    // check for deprecated MarkerSymbol type.
    if ( !symbol.valid() )
    {
        if ( _style.has<MarkerSymbol>() )
            symbol = _style.get<MarkerSymbol>()->convertToInstanceSymbol();
    }

    if ( !symbol.valid() )
    {
        OE_WARN << LC << "No appropriate symbol found in stylesheet; aborting." << std::endl;
        return 0L;
    }

    // establish the resource library, if there is one:
    _resourceLib = 0L;

    const StyleSheet* sheet = context.getSession() ? context.getSession()->styles() : 0L;

    if ( sheet && symbol->library().isSet() )
    {
        _resourceLib = sheet->getResourceLibrary( symbol->library()->expr() );

        if ( !_resourceLib.valid() )
        {
            OE_WARN << LC << "Unable to load resource library '" << symbol->library()->expr() << "'"
                << "; may not find instance models." << std::endl;
        }
    }

    // reset this marker:
    _normalScalingRequired = false;

    // Compute localization info:
    FilterContext newContext( context );

    computeLocalizers( context );

    osg::Group* group = createDelocalizeGroup();

    osg::ref_ptr< osg::Group > attachPoint = new osg::Group;
    group->addChild(attachPoint.get());

    // Process the feature set, using clustering if requested
    bool ok = true;

    process( features, symbol, context.getSession(), attachPoint.get(), newContext );
    if (_cluster)
    {
        // Extract the unclusterable things
        osg::ref_ptr< osg::Node > unclusterables = extractUnclusterables(attachPoint);

        // We run on the attachPoint instead of the main group so that we don't lose the double precision declocalizer transform.
        MeshFlattener::run(attachPoint);

        // Add the unclusterables back to the attach point after the rest of the graph was flattened.
        if (unclusterables.valid())
        {
            attachPoint->addChild(unclusterables);
        }
    }

    // return proper context
    context = newContext;

#if 0
    // TODO: OBE due to shader pipeline
    // see if we need normalized normals
    if ( _normalScalingRequired )
    {
        // TODO: carefully test for this, since GL_NORMALIZE hurts performance in 
        // FFP mode (RESCALE_NORMAL is faster for uniform scaling); and I think auto-normal-scaling
        // is disabled entirely when using shaders. For now I believe we are dropping to FFP
        // when not using instancing ...so just check for that
        if ( !_useDrawInstanced )
        {
            group->getOrCreateStateSet()->setMode( GL_NORMALIZE, osg::StateAttribute::ON );
        }
    }
#endif

    //osgDB::writeNodeFile(*group, "c:/temp/clustered.osg");

    return group;
}
