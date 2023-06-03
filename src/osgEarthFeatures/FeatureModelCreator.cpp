/* --*-c++-*-- */
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

#include <osgEarthFeatures/FeatureModelCreator>
#include <osgEarthFeatures/CropFilter>
#include <osgEarthFeatures/FeatureSourceIndexNode>
#include <osgEarthFeatures/Session>

#include <osgEarth/Map>
#include <osgEarth/Capabilities>
#include <osgEarth/Clamping>
#include <osgEarth/ClampableNode>
#include <osgEarth/CullingUtils>
#include <osgEarth/ElevationLOD>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FadeEffect>
#include <osgEarth/NodeUtils>
#include <osgEarth/Registry>
#include <osgEarth/ThreadingUtils>

#include <osg/CullFace>
#include <osg/PagedLOD>
#include <osg/ProxyNode>
#include <osgDB/FileNameUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/WriteFile>
#include <osgUtil/Optimizer>

#include <algorithm>
#include <iterator>

#define LC "[FeatureModelCreator] " << getName() << ": "

using namespace osgEarth;
using namespace osgEarth::Features;
using namespace osgEarth::Symbology;

#undef USE_PROXY_NODE_FOR_TESTING
#define OE_TEST OE_NULL
//#define OE_TEST OE_NOTICE


//---------------------------------------------------------------------------
FeatureModelCreator::FeatureModelCreator(
    Session* session,
    FeatureNodeFactory* factory,
    FeatureSourceIndex* featureIndex) :_session(session), _factory(factory), _featureIndex(featureIndex), 
                                       _Profile(nullptr), _ReadOptions(nullptr), _FeatureSource(nullptr), _ShadPol(SHADERPOLICY_GENERATE), _context(nullptr), _compiler(nullptr)

{
    GeomFeatureNodeFactory *realFactory = (GeomFeatureNodeFactory *)_factory.get();
    realFactory->getOptions(_GeometryOptions);
}



FeatureModelCreator::~FeatureModelCreator()
{
//    osgEarthFeatureModelPseudoLoader::unregisterGraph(_uid);
    if(_context)
        delete _context;
    if(_compiler)
        delete _compiler;
}



bool
FeatureModelCreator::createOrUpdateNode(Feature * cursor, const Style& style, FilterContext& context, const osgDB::Options* readOptions, osg::ref_ptr<osg::Node>& output)
{
    output = _compiler->ExtrudeAFeature(cursor);
    return output.valid();
}


osg::Group*
FeatureModelCreator::createStyleGroup(const Style& style, Feature * workingSet, const FilterContext& contextPrototype, const osgDB::Options* readOptions)
{
    osg::Group* styleGroup = 0L;

    OE_DEBUG << LC << "Created style group \"" << style.getName() << "\"\n";

    FilterContext context(contextPrototype);

    // First Crop the feature set to the working extent.
    // Note: There is an obscure edge case that can happen is a feature's centroid
    // falls exactly on the crop extent boundary. In that case the feature can
    // show up in more than one tile. It's rare and not trivial to mitigate so for now
    // we have decided to do nothing. :)

    // finally, compile the features into a node.
    osg::ref_ptr<osg::Node> node;
    if (createOrUpdateNode(workingSet, style, context, readOptions, node))
    {
        if (!styleGroup)
            styleGroup = getOrCreateStyleGroupFromFactory(style);

        // if it returned a node, add it. (it doesn't necessarily have to)
        if (node.valid())
            styleGroup->addChild(node.get());
    }

    return styleGroup;
}


osg::Group*
FeatureModelCreator::getOrCreateStyleGroupFromFactory(const Style& style)
{
    osg::Group* styleGroup = _factory->getOrCreateStyleGroup(style, _session.get());

    // Check the style and see if we need to active GPU clamping. GPU clamping
    // is currently all-or-nothing for a single FMG.
    // Warning. This needs attention w.r.t. caching, since the "global" styles don't cache. -gw
//    checkForGlobalStyles(style);

    // Apply render symbology at the style group level.
//    applyRenderSymbology(style, styleGroup);

    return styleGroup;
}

void FeatureModelCreator::Init(const GeoExtent& extent, FeatureProfile* Profile, const osgDB::Options* readOptions)
{
    _extent = extent;
    _Profile = Profile;
    _ReadOptions = readOptions;
    _FeatureSource = _session->getFeatureSource();
    if (_session->styles()->selectors().size() == 0)
    {
        // attempt to glean the style from the feature source name:
        _defaultStyle = *_session->styles()->getStyle(
            *_session->getFeatureSource()->getFeatureSourceOptions().name());
    }
    std::string optionsString = readOptions->getOptionString();
    if (optionsString.find("DisableShaders") != std::string::npos)
    {
        _ShadPol = SHADERPOLICY_DISABLE;
    }

    FeatureSourceIndexNode* index = 0L;

    if (_FeatureSource)
    {
        if (_featureIndex.valid())
        {
            index = new FeatureSourceIndexNode(_featureIndex.get());
        }
    }

    _context = new FilterContext(_session.get(), _Profile, _extent, index, _ShadPol);

    _compiler = new GeometryCompiler(_GeometryOptions);
    _compiler->InitForTile(_defaultStyle, *_context);

}

bool FeatureModelCreator::buildWorkingSet(osg::ref_ptr<osg::Group>& group, Feature * WorkingSet)
{
    // set up for feature indexing if appropriate:


    if (_FeatureSource)
    {
        if (_featureIndex.valid())
        {
            group = new FeatureSourceIndexNode(_featureIndex.get());
        }
    }

    if (!group.valid())
    {
        group = new osg::Group();
    }

    osg::Group*  styleGroup = createStyleGroup(_defaultStyle, WorkingSet, *_context, _ReadOptions);

    if (styleGroup && !group->containsNode(styleGroup))
        group->addChild(styleGroup);

    return group->getNumChildren() > 0;
}

