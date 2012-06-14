#include "ParticleGeode.hpp"

namespace vizkit {

std::vector<osg::Vec4> ParticleGeode::color_map;

ParticleGeode::ParticleGeode() : Geode()
{
    color = new osg::Vec4Array;
    vertices = new osg::Vec3Array;
    geom = new osg::Geometry;

    sonar_color = new osg::Vec4Array;
    sonar_vertices = new osg::Vec3Array;
    sonar_geom = new osg::Geometry;
    sonar_draw = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4);

    desire_color = new osg::Vec4Array;
    desire_vertices = new osg::Vec3Array;
    desire_geom = new osg::Geometry;
    desire_draw = new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 8);

    real_point_show = false;
    desire_point_show = false;

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 24));
    geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    sonar_geom->addPrimitiveSet(sonar_draw.get());
    sonar_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    sonar_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    desire_geom->addPrimitiveSet(desire_draw.get());
    desire_geom->setColorBinding(osg::Geometry::BIND_OVERALL);
    desire_geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    this->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    this->addDrawable(geom.get());
    
    changed = true;
}

osg::Vec4& ParticleGeode::gradient(double weight) 
{
    if(std::isnan(weight))
        weight = 0.0;

    if(color_map.size() == 0) {
        for(unsigned i = 0; i < 256; i++) {
            color_map.push_back(osg::Vec4(1.0f, i / 255.0f, 0.0f, 0.7f));
        }
    }

    return color_map.at(floor(weight * 255));
}

ParticleGeode::~ParticleGeode()
{
}

void ParticleGeode::render()
{
    if(!changed)
        return;

    switch(type) {
        case BOX:
            renderAsBox(particle, size, scale);
            break;
        case LINE:
            renderAsLine(particle, max_z, min_z, size, scale);
            break;
    }

    renderRealPoint();
    renderDesirePoint();

    changed = false;
}


void ParticleGeode::renderAsBox(const uw_localization::Particle& particle, double size, double scale)
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


void ParticleGeode::renderAsLine(const uw_localization::Particle& particle, double from, double to, double size, double scale)
{
    vertices->clear();
    color->clear();

    base::Vector3d pos = particle.position;
    double height = (to - from) * (particle.main_confidence / scale);
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

void ParticleGeode::renderDesirePoint()
{
    desire_color->clear();
    desire_vertices->clear();

    if(!desire_point_show) {
        this->removeDrawable(desire_geom.get());
        return;
    }

    if(!this->containsDrawable(desire_geom.get()))
        this->addDrawable(desire_geom.get());

    osg::Vec3 pos(particle.position.x(), particle.position.y(), particle.position.z());
    osg::Vec3 des(sonar.desire_point.x(), sonar.desire_point.y(), 0.0f);

    desire_vertices->push_back(des + osg::Vec3(-0.1f, 0.0f, min_z));
    desire_vertices->push_back(des + osg::Vec3(0.1f, 0.0f, min_z));
    desire_vertices->push_back(des + osg::Vec3(0.1f, 0.0f, max_z));
    desire_vertices->push_back(des + osg::Vec3(-0.1f, 0.0f, max_z));

    desire_vertices->push_back(des + osg::Vec3(0.0f, -0.1f, min_z));
    desire_vertices->push_back(des + osg::Vec3(0.0f, 0.1f, min_z));
    desire_vertices->push_back(des + osg::Vec3(0.0f, 0.1f, max_z));
    desire_vertices->push_back(des + osg::Vec3(0.0f, -0.1f, max_z));

    desire_color->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 0.7f));
    desire_geom->setVertexArray(desire_vertices.get());
    desire_geom->setColorArray(desire_color.get());
}

void ParticleGeode::renderRealPoint()
{
    sonar_color->clear();
    sonar_vertices->clear();

    if(!real_point_show) {
        this->removeDrawable(sonar_geom.get());
        return;
    }

    if(!this->containsDrawable(sonar_geom.get()))
        this->addDrawable(sonar_geom.get());

    osg::Vec3 pos(particle.position.x(), particle.position.y(), particle.position.z());
    osg::Vec3 des(sonar.real_point.x(), sonar.real_point.y(), 0.0f);

    sonar_vertices->push_back(pos);
    sonar_vertices->push_back(pos);
    sonar_vertices->push_back(des + osg::Vec3(0.0f, 0.0f, min_z));
    sonar_vertices->push_back(des + osg::Vec3(0.0f, 0.0f, max_z));

    sonar_color->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 0.7f));
    sonar_geom->setVertexArray(sonar_vertices.get());
    sonar_geom->setColorArray(sonar_color.get());
}

void ParticleGeode::updateParticle(const uw_localization::Particle& p)
{
    particle = p;
    changed = true;
}


void ParticleGeode::updateSonar(const uw_localization::PointInfo& info) 
{
    sonar = info;
    real_point_show = false;
    desire_point_show = false;
    changed = true;
}

void ParticleGeode::showRealPoint(bool show)
{
    real_point_show = show;
    changed = true;
}


void ParticleGeode::showDesirePoint(bool show)
{
    desire_point_show = show;
    changed = true;
}


void ParticleGeode::setViz(VizType type_, double min_z_, double max_z_, double size_, double scale_)
{
    type = type_;
    min_z = min_z_;
    max_z = max_z_;
    size = size_;
    scale = scale_;
}

ParticleGeode::VizType ParticleGeode::type = LINE;
double ParticleGeode::min_z = 0.0;
double ParticleGeode::max_z = 1.0;
double ParticleGeode::size  = 0.1;
double ParticleGeode::scale = 1.0;

}
