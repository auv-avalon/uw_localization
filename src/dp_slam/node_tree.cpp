#include "node_tree.hpp"

using namespace uw_localization;


ParticleNode::ParticleNode(ParticleNode* parent_node){
  
  parent = parent_node;
  child_l = 0;
  child_r = 0;  
  
}


ParticleNode::~ParticleNode(){
  
  delete child_l;
  delete child_r;  
}

void ParticleNode::insert(ParticleNode* child){
 
  if(child->val < val){
    
    if(child_l != 0){
      child_l->insert(child);
    }else
    {
      child_l = child;
      child->setParent(this);
    }
  }else{
    
    if(child_r != 0){
      child_r->insert(child);
    }else
    {
      child_r = child;
      child->setParent(this);
  }
  
}


void ParticleNode::erase(){  
  
  if(parent != 0){
    parent->eraseChild(this);
    
    
    if(child_l == 0 && child_r == 0){
      
    }else if(child_l == 0){
      parent->insert(child_r);
    
    }else if(child_r == 0){
      parent->insert(child_l);
    
    
    }else{
      
      child_r->insert(child_l);
      parent->insert(child_r);
      
    }
    
    child_l = 0;
    child_r = 0;  
    
  }
  else if(root != 0){
    root->setRoot(0);
        
    if(child_l == 0 && child_r == 0){
      
    }else if(child_l == 0){
      root->setRoot(child_r);
      child_r->setParent(0);
      child_r->setRoot(root);
    
    }else if(child_r == 0){
      root->setRoot(child_l);
      child_l->setParent(0);
      child_l->setRoot(root);
    
    }else{
      
      child_r->insert(child_l);
      root->setRoot(child_r);
      child_r->setRoot(root);
      
    }
    
    child_l = 0;
    child_r = 0;  
    
  }  
}


void ParticleNode::eraseChild(ParticleNode* child){
  
  if(child_l == child)
    child_l = 0;
  else if(child_r == child)
    child_r = 0;
  
}


ParticleNode* ParticleNode::search(double v){
 
  if(v == val)
    return this;  
  
  if(v < val && child_l != 0)
  {
    return child_l->search(val);
  }
  else if(v > val && child_r != 0)
  {
    return child_r->search(val);
  }  
  
  return 0;
  
}


void ParticleNode::setRoot(NodeTree* root){
  this->root = root;
}


NodeTree::NodeTree(){
 root = 0;
  
}

NodeTree::~NodeTree(){
  delete root;
}


void NodeTree::setRoot(ParticleNode* node){
  root = node;
}

/*
void NodeTree::insert(ParticleNode* node){
  
}*/



bool operator<(ParticleNode const& lhs, ParticleNode const& rhs){
  
  return true; //TODO
}
