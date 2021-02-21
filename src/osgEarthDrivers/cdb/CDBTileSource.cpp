// Copyright (c) 1994-2013 Georgia Tech Research Corporation, Atlanta, GA
// This file is part of FalconView(tm).

// FalconView(tm) is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// FalconView(tm) is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public License
// along with FalconView(tm).  If not, see <http://www.gnu.org/licenses/>.

// FalconView(tm) is a trademark of Georgia Tech Research Corporation.

// 2014-2015 GAJ Geospatial Enterprises, Orlando FL
// Modified for General Incorporation of Common Database (CDB) support within osgEarth
// CDBTileSource.cpp
//
// 2016-2017 Visual Awareness Technologies and Consulting Inc. St Petersburg FL


#include <osgEarth/Registry>
#include <osgEarth/URI>
#include <osgEarth/TileSource>
#include <osgEarth/ImageToHeightFieldConverter>

#ifdef _WIN32
#include <Windows.h>
#endif

#include "CDBTileSource"
#include "CDBOptions"
#include <CDB_TileLib/CDB_Tile>


using namespace osgEarth;


CDBTileSource::CDBTileSource( const osgEarth::TileSourceOptions& options ) : TileSource(options), CDBOptions(options), _UseCache(false), _rootDir(""), _cacheDir(""), 
																			_tileSize(1024), _dataSet("_S001_T001_"), _Be_Verbose(false), _LightMap(false), _Materials(false),
																			_MaterialMask(false)

{

}   


// CDB uses unprojected lat/lon
Status CDBTileSource::initialize(const osgDB::Options* dbOptions)
{
   // No caching of source tiles
   //Note: This is in reference to osgEarth Cacheing. 
   //This driver provides the capablity to cache CDB lower levels of detail. If osgearth caching is used 
   //Then CDB cacheing should not.
   _dbOptions = osgEarth::Registry::instance()->cloneOrCreateOptions( dbOptions );
//   osgEarth::CachePolicy::NO_CACHE.apply( _dbOptions.get() );
   // Make sure the root directory is set

   bool errorset = false;
   std::string Errormsg = "";
   //The CDB Root directory is required.
   if (!rootDir().isSet())
   {
	   OE_WARN << "CDB root directory not set!" << std::endl;
	   Errormsg = "CDB root directory not set";
	   errorset = true;
   }
   else
   {
	   _rootDir = rootDir().value();
   }

   //Find a jpeg2000 driver for the image layer.
   if (!CDB_Tile::Initialize_Tile_Drivers(Errormsg))
   {
	   errorset = true;
   }

   if (DisableBathemetry().isSet())
   {
	   bool disable = DisableBathemetry().value();
	   if (disable)
		   CDB_Tile::Disable_Bathyemtry(true);
   }

   if (Verbose().isSet())
   {
	   bool verbose = Verbose().value();
	   if (verbose)
		   _Be_Verbose = true;
   }
   //Get the chache directory if it is set and turn on the cacheing option if it is present
   if (cacheDir().isSet())
   {
	   _cacheDir = cacheDir().value();
	   _UseCache = true;
   }
   
   if (LightMap().isSet())
   {
	   _LightMap = LightMap().value();
  }

   if (Materials().isSet())
   {
	   _Materials = Materials().value();
   }

   if (MaterialMask().isSet())
   {
	   _MaterialMask = MaterialMask().value();
  }

   //verify tilesize
   if (tileSize().isSet())
	   _tileSize = tileSize().value();

   bool profile_set = false;
   int maxcdbdatalevel = 14;
   //Check if there are limits on the maximum Lod to use
   if (MaxCDBLevel().isSet())
   {
	   maxcdbdatalevel = MaxCDBLevel().value();
   }

   //Check to see if we have been told how many negitive lods to use
   int Number_of_Negitive_LODs_to_Use = 0;
   if (NumNegLODs().isSet())
   {
	   Number_of_Negitive_LODs_to_Use = NumNegLODs().value();
   }
 
   //Check to see if we are loading only a limited area of the earth
   if (Limits().isSet())
   {
	   std::string cdbLimits = Limits().value();
	   double	min_lon,
				max_lon,
				min_lat,
				max_lat;

	   int count = sscanf(cdbLimits.c_str(), "%lf,%lf,%lf,%lf", &min_lon, &min_lat, &max_lon, &max_lat);
	   if (count == 4)
	   {
		   //CDB tiles always filter to geocell boundaries

		   min_lon = round(min_lon);
		   min_lat = round(min_lat);
		   max_lat = round(max_lat);
		   max_lon = round(max_lon);
		   //Make sure the profile includes hole tiles above and below 50 deg
		   int lonstep = CDB_Tile::Get_Lon_Step(min_lat);
		   lonstep = CDB_Tile::Get_Lon_Step(max_lat) > lonstep ? CDB_Tile::Get_Lon_Step(max_lat) : lonstep;
		   if (lonstep > 1)
		   {
			   int delta = (int)min_lon % lonstep;
			   if (delta)
			   {
				   min_lon = (double)(((int)min_lon - lonstep) * lonstep);
			   }
			   delta = (int)max_lon % lonstep;
			   if (delta)
			   {
				   max_lon = (double)(((int)max_lon + lonstep) * lonstep);
			   }
		   }
		   //Expand the limits if necessary to meet the criteria for the number of negitive lods specified
		   int subfact = 2 << Number_of_Negitive_LODs_to_Use;  //2 starts with lod 0 this means howerver a minumum of 4 geocells will be requested even if only one
		   if ((max_lon > min_lon) && (max_lat > min_lat))	   //is specified in the limits section of the earth file.
		   {
			   unsigned tiles_x = (unsigned)(max_lon - min_lon);
			   int modx = tiles_x % subfact;
			   if (modx != 0)
			   {
				   tiles_x = ((tiles_x + subfact) / subfact) * subfact;
				   max_lon = min_lon + (double)tiles_x;
			   }
			   tiles_x /= subfact;

			   unsigned tiles_y = (unsigned)(max_lat - min_lat);
			   int mody = tiles_y % subfact;
			   if (mody != 0)
			   {
				   tiles_y = ((tiles_y + subfact) / subfact) * subfact;
				   max_lat = min_lat + (double)tiles_y;
			   }
			   tiles_y /= subfact;

			   //Create the Profile with the calculated limitations
			   osg::ref_ptr<const SpatialReference> src_srs;
			   src_srs = SpatialReference::create("EPSG:4326");
			   GeoExtent extents = GeoExtent(src_srs, min_lon, min_lat, max_lon, max_lat);
			   getDataExtents().push_back(DataExtent(extents, 0, maxcdbdatalevel+Number_of_Negitive_LODs_to_Use+1)); //plus number of sublevels
			   setProfile(osgEarth::Profile::create(src_srs, min_lon, min_lat, max_lon, max_lat, tiles_x, tiles_y));

			   OE_INFO "CDB Profile Min Lon " << min_lon << " Min Lat " << min_lat << " Max Lon " << max_lon << " Max Lat " << max_lat << "Tiles " << tiles_x << " " << tiles_y << std::endl;
			   OE_INFO "  Number of negitive lods " << Number_of_Negitive_LODs_to_Use << " Subfact " << subfact << std::endl;
			   profile_set = true;
		   }
	   }
	   if (!profile_set)
		   OE_WARN << "Invalid Limits received by CDB Driver: Not using Limits" << std::endl;

   }

	   // Always a WGS84 unprojected lat/lon profile.
   if (!profile_set)
   {
	   //Use a default world profile
	   Number_of_Negitive_LODs_to_Use = 5;
	   osg::ref_ptr<const SpatialReference> src_srs;
	   src_srs = SpatialReference::create("EPSG:4326");
	   GeoExtent extents = GeoExtent(SpatialReference::create("EPSG:4326"), -180.0, -90.0, 180.0, 90.0);
	   getDataExtents().push_back(DataExtent(extents, 0, maxcdbdatalevel + Number_of_Negitive_LODs_to_Use));
//	   setProfile(osgEarth::Profile::create(src_srs, -180.0, -102.0, 204.0, 90.0, 6U, 3U));
	   setProfile(osgEarth::Profile::create(src_srs, -180.0, -102.0, 204.0, 90.0, 12U, 6U));
	   if (!_UseCache)
	   {
		   if (!errorset)
		   {
			   //Look for the Default Cache Dir
			   std::stringstream buf;
			   buf << _rootDir
				   << "/osgEarth"
				   << "/CDB_Cache";
			   _cacheDir = buf.str();
#ifdef _WIN32
			   DWORD ftyp = ::GetFileAttributes(_cacheDir.c_str());
			   if (ftyp != INVALID_FILE_ATTRIBUTES)
			   {
				   _UseCache = true;
			   }
#else
			   int ftyp = ::access(_cacheDir.c_str(), F_OK);
			   if (ftyp == 0)
			   {
				   _UseCache = true;
			   }

#endif
		   }
	   }
   }

   if (errorset)
   {
	   Status Rstatus(Errormsg);
	   return Rstatus;
   }
   else
	   return STATUS_OK;
}


osg::Image* CDBTileSource::createImage(const osgEarth::TileKey& key,
										osgEarth::ProgressCallback* progress )
{

	osg::Image *ret_Image = NULL;

	const GeoExtent key_extent = key.getExtent();
	CDB_Tile_Type tiletype = Imagery;
	CDB_Tile_Extent tileExtent(key_extent.north(), key_extent.south(), key_extent.east(), key_extent.west());
	CDB_Tile *mainTile = new CDB_Tile(_rootDir, _cacheDir, tiletype, _dataSet, &tileExtent, _LightMap, _Materials, _MaterialMask);
	std::string base = mainTile->FileName();
	int cdbLod = mainTile->CDB_LOD_Num();


	if (cdbLod >= 0)
	{
		if (CDB_Tile::Get_Lon_Step(tileExtent.South) == 1.0)
		{
			if (mainTile->Tile_Exists())
			{
				if (_Be_Verbose)
				{
					if (!_Materials && !_LightMap)
						OSG_WARN << "Imagery: Loading " << base << std::endl;
					else if (_Materials)
						OSG_WARN << "Imagery: Loading " << mainTile->Subordinate2_Name() << std::endl;
					else if (_LightMap)
						OSG_WARN << "Imagery: Loading " << mainTile->Subordinate_Name() << std::endl;
				}
				mainTile->Load_Tile();
				ret_Image = mainTile->Image_From_Tile();
			}
			else
			{
				if (_Be_Verbose)
					OSG_WARN << "Imagery: Blacklisting " << base << std::endl;
				Registry::instance()->blacklist(base);
			}
		}
		else
		{
			if (mainTile->Build_Earth_Tile())
			{
				if (_Be_Verbose)
				{
					if (!_Materials && !_LightMap)
						OSG_WARN << "Imagery: Loading " << base << std::endl;
					else if (_Materials)
						OSG_WARN << "Imagery: Loading " << mainTile->Subordinate2_Name() << std::endl;
					else if (_LightMap)
						OSG_WARN << "Imagery: Loading " << mainTile->Subordinate_Name() << std::endl;
				}
				OE_DEBUG "Imagery Built Earth Tile " << key.str() << "=" << base << std::endl;
				ret_Image = mainTile->Image_From_Tile();
			}
		}
	}
	else
	{
		if (mainTile->Tile_Exists())
		{
			if (_Be_Verbose)
			{
				if (!_Materials && !_LightMap)
					OSG_WARN << "Imagery: Loading " << base << std::endl;
				else if (_Materials)
					OSG_WARN << "Imagery: Loading " << mainTile->Subordinate2_Name() << std::endl;
				else if (_LightMap)
					OSG_WARN << "Imagery: Loading " << mainTile->Subordinate_Name() << std::endl;
			}
			mainTile->Load_Tile();
			ret_Image = mainTile->Image_From_Tile();
		}
		else
		{
			if (mainTile->Build_Cache_Tile(_UseCache))
			{
				if (_Be_Verbose)
				{
					if (!_Materials && !_LightMap)
						OSG_WARN << "Imagery: Loading " << base << std::endl;
					else if (_Materials)
						OSG_WARN << "Imagery: Loading " << mainTile->Subordinate2_Name() << std::endl;
					else if (_LightMap)
						OSG_WARN << "Imagery: Loading " << mainTile->Subordinate_Name() << std::endl;
				}
				ret_Image = mainTile->Image_From_Tile();
			}
		}
	}
	delete mainTile;

#ifdef _DEBUG
	if (ret_Image)
		OE_INFO "Imagery " << key.str() << "=" << base << std::endl;
	else
		OE_INFO "Missing Imagery " << key.str() << "=" << base << std::endl;
#endif


	return ret_Image;

}

osg::HeightField* CDBTileSource::createHeightField(const osgEarth::TileKey& key,
   osgEarth::ProgressCallback* progress )
{

	osg::HeightField* ret_Field = NULL;

	const GeoExtent key_extent = key.getExtent();
	CDB_Tile_Type tiletype = Elevation;
	CDB_Tile_Extent tileExtent(key_extent.north(), key_extent.south(), key_extent.east(), key_extent.west());
	CDB_Tile *mainTile = new CDB_Tile(_rootDir, _cacheDir, tiletype, _dataSet, &tileExtent, _LightMap, _Materials, _MaterialMask);
	std::string base = mainTile->FileName();
	int cdbLod = mainTile->CDB_LOD_Num();

	if (cdbLod >= 0)
	{
		if (CDB_Tile::Get_Lon_Step(tileExtent.South) == 1.0)
		{
			if (mainTile->Tile_Exists())
			{
				if (_Be_Verbose)
					OSG_WARN << "Elevation: Loading " << base << std::endl;
				mainTile->Load_Tile();
				ret_Field = mainTile->HeightField_From_Tile();
			}
			else
			{
				if (_Be_Verbose)
					OSG_WARN << "Elevation: Blacklisting " << base << std::endl;
				Registry::instance()->blacklist(base);
			}
		}
		else
		{
			if (mainTile->Build_Earth_Tile())
			{
				ret_Field = mainTile->HeightField_From_Tile();
				OE_DEBUG "Elevation Built Earth Tile " << key.str() << "=" << base << std::endl;
			}
		}
	}
	else
	{
		if (mainTile->Tile_Exists())
		{
			if (_Be_Verbose)
				OSG_WARN << "Elevation: Loading " << base << std::endl;
			mainTile->Load_Tile();
			ret_Field = mainTile->HeightField_From_Tile();
		}
		else
		{
			if (mainTile->Build_Cache_Tile(_UseCache))
			{
				if (_Be_Verbose)
					OSG_WARN << "Elevation: Built Cache tile " << base << std::endl;
				ret_Field = mainTile->HeightField_From_Tile();
			}
		}
	}
	delete mainTile;

#ifdef _DEBUG
	if (ret_Field)
		OE_INFO "Elevation " << key.str() << "=" << base << std::endl;
	else
		OE_INFO "Missing Elevation " << key.str() << "=" << base << std::endl;
#endif

	return ret_Field;
}

std::string CDBTileSource::getExtension()  const 
{
   return "jp2";
}

/** Tell the terrain engine not to cache tiles from this source. */
osgEarth::CachePolicy CDBTileSource::getCachePolicyHint() const
{
   return osgEarth::CachePolicy::NO_CACHE;
}
