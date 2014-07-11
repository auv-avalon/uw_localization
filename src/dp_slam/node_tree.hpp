#ifndef UW_LOCALIZATION_DPSLAM_NODETREE_HPP
#define UW_LOCALIZATION_DPSLAM_NODETREE_HPP

#include "../types/particle.hpp"

namespace uw_localization{
 
  class NodeTree;
  
 class ParticleNode{
   
   ParticleNode* parent;
   ParticleNode* child_l;
   ParticleNode* child_r;
   NodeTree* root;
      
   double val;
   
 public:
   
   ParticleNode(ParticleNode* parent_node, NodeTree* root, PoseSlamParticle particle);
   ~ParticleNode();
   
   void insert(ParticleNode* child);
   void erase();
   void eraseChild(ParticleNode* child);
   void setRoot(NodeTree* root);
   void setParent(ParticleNode* parent);
   
   ParticleNode* search(double v);
   
   friend bool operator<(ParticleNode const& lhs, ParticleNode const& rhs); 
   
 };
  
  
 class NodeTree{
   
 public:
   NodeTree();
   ~NodeTree();
   
   void setRoot(ParticleNode* node);
   
 private:   
   ParticleNode* root;
   
   
 };
  
  
}

#endif