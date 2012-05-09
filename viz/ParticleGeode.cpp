#include "ParticleGeode.hpp"

namespace vizkit {

std::vector<osg::Vec4> ParticleGeode::color_map;

ParticleGeode:: ParticleGeode() : Geode()
{
    color = new osg::Vec4Array;
    vertices = new osg::Vec3Array;
    geom = new osg::Geometry;

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 24));
    geom->setVertexArray(vertices.get());
    geom->setColorArray(color.get());
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    this->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    this->addDrawable(geom.get());
}

osg::Vec4& ParticleGeode::gradient(double weight) 
{
    if(std::isnan(weight))
        weight = 0.0;

    if(color_map.size() == 0) {
        for(unsigned i = 0; i < 256; i++) {
            if(i < 128)
                color_map.push_back(osg::Vec4(1.0f, i / 127.0f, 0.0f, 0.7f));
            else
                color_map.push_back(osg::Vec4(1.0f - ((i - 128) / 127.0f), 1.0f, 0.0f, 0.7f));
        }
    }

    return color_map.at(floor(weight * 255));
}

ParticleGeode::~ParticleGeode()
{
}

 
void ParticleGeode::update_as_box(const uw_localization::Particle& particle, double size, double scale)
{
    vertices->clear();
    color->clear();

    base::Vector3d pos = particle.position;
    float offset = size / 2.0f;

    // front
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, pos.z() + offset));

    // back
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, pos.z() - offset));

    // top
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, pos.z() + offset));

    // bottom
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, pos.z() + offset));

    // left
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, pos.z() + offset));

    // right
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, pos.z() - offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, pos.z() + offset));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, pos.z() + offset));

    color->push_back(gradient(particle.main_confidence / scale));
    
    geom->setVertexArray(vertices.get());
    geom->setColorArray(color.get());
}


void ParticleGeode::update_as_line(const uw_localization::Particle& particle, double from, double max_height, double size, double scale)
{
    vertices->clear();
    color->clear();

    base::Vector3d pos = particle.position;
    double height = max_height * (particle.main_confidence / scale);
    float offset = size / 2.0f;

    // top
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, from + height));

    // bottom
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, from));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, from));

    // left
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, from + height));

    // right
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, from + height));

    // back
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, from));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, from));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() + offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() - offset, pos.y() - offset, from + height));

    // fron
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, from));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() + offset, from + height));
    vertices->push_back(osg::Vec3(pos.x() + offset, pos.y() - offset, from + height));

    color->push_back(gradient(particle.main_confidence / scale));
    
    geom->setVertexArray(vertices.get());
    geom->setColorArray(color.get());
}



}
