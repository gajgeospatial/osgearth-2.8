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
#include <osgEarthSymbology/ResourceCache>
#include <osgDB/Archive>

using namespace osgEarth;
using namespace osgEarth::Symbology;


// internal thread-safety not required since we mutex it in this object.
ResourceCache::ResourceCache() : // const osgDB::Options* dbOptions ) :
//_dbOptions    ( dbOptions ),
_skinCache    ( false ),
_instanceCache( false ),
_resourceLibraryCache( false )
{
    //nop
}

bool
ResourceCache::getOrCreateStateSet(SkinResource*                skin,
                                   osg::ref_ptr<osg::StateSet>& output,
                                   const osgDB::Options*        readOptions)
{
    output = 0L;
    //std::string key = skin->getConfig().toJSON(false);

    // Note: we use the imageURI as the basis for the caching key since 
    // it's the only property used by Skin->createStateSet(). If that
    // changes, we need to address it here. It might be better it SkinResource
    // were to provide a unique key.
    std::string key = skin->getUniqueID();

    // exclusive lock (since it's an LRU)
    {
        Threading::ScopedMutexLock exclusive( _skinMutex );
            
        // double check to avoid race condition
        SkinCache::Record rec;       
        if ( _skinCache.get(key, rec) && rec.value().valid() )
        {
            output = rec.value().get();
        }
        else
        {
            // still not there, make it.
            output = skin->createStateSet(readOptions);
            if ( output.valid() && !skin->materialURI().isSet())
            {
                _skinCache.insert( key, output.get() );
            }
        }
    }

    return output.valid();
}

bool
ResourceCache::getOrCreateMatStateSet(SkinResource*                skin,
									 osg::ref_ptr<osg::StateSet>& output,
	                                 const osgDB::Options*        readOptions)
{
	//std::string key = skin->getConfig().toJSON(false);

	// Note: we use the imageURI as the basis for the caching key since 
	// it's the only property used by Skin->createStateSet(). If that
	// changes, we need to address it here. It might be better it SkinResource
	// were to provide a unique key.
	std::string key = skin->getUniqueID();

	// exclusive lock (since it's an LRU)
	{
		Threading::ScopedMutexLock exclusive(_skinMutex);

		// double check to avoid race condition
		SkinCache::Record rec;
		if (_skinCache.get(key, rec) && rec.value().valid())
		{
			output = rec.value().get();
			int numTexModes = output->getNumTextureModeLists();
			if(numTexModes < 2)
				skin->createMatStateSet( output, readOptions);
		}
		else
		{
			// still not there, make it.
			skin->createMatStateSet(output, readOptions);
			if (output.valid())
			{
				_skinCache.insert(key, output.get());
			}
		}
	}

	return output.valid();
}

bool
ResourceCache::getOrCreateInstanceNode(InstanceResource*        res,
                                       osg::ref_ptr<osg::Node>& output,
                                       const osgDB::Options*    readOptions)
{
    output = 0L;
    std::string key = res->getConfig().toJSON(false);

    // exclusive lock (since it's an LRU)
    {
        Threading::ScopedMutexLock exclusive( _instanceMutex );

        // double check to avoid race condition
        InstanceCache::Record rec;
        if ( _instanceCache.get(key, rec) && rec.value().valid() )
        {
            output = rec.value().get();
        }
        else
        {
            // still not there, make it.
            output = res->createNode(readOptions);
            if ( output.valid() )
            {
                _instanceCache.insert( key, output.get() );
            }
        }
    }

    return output.valid();
}

bool
ResourceCache::cloneOrCreateInstanceNode(InstanceResource*        res,
                                         osg::ref_ptr<osg::Node>& output,
                                         const osgDB::Options*    readOptions, osgDB::Archive *ar)
{
    output = 0L;
    std::string key = res->getConfig().toJSON(false);

    // exclusive lock (since it's an LRU)
    {
        Threading::ScopedMutexLock exclusive( _instanceMutex );

        // Deep copy everything except for images.  Some models may share imagery so we only want one copy of it at a time.
        osg::CopyOp copyOp = osg::CopyOp::DEEP_COPY_ALL & ~osg::CopyOp::DEEP_COPY_IMAGES & ~osg::CopyOp::DEEP_COPY_TEXTURES;

        // double check to avoid race condition
        InstanceCache::Record rec;
        if ( _instanceCache.get(key, rec) && rec.value().valid() )
        {
            output = osg::clone(rec.value().get(), copyOp);
        }
        else
        {
            // still not there, make it.
			if (ar)
			{
				std::string modelname = res->uri().get().base();
				osgDB::ReaderWriter::ReadResult r = ar->readNode(modelname, readOptions);
				if (r.validNode())
					output = r.getNode();
			}
			else
			{
           		output = res->createNode(readOptions);
			}
            if ( output.valid() )
            {
                _instanceCache.insert( key, output.get() );
                output = osg::clone(output.get(), copyOp);
            }
        }
    }

    return output.valid();
}
