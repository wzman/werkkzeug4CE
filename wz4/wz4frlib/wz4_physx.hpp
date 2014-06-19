/*+**************************************************************************/
/***                                                                      ***/
/***   This file is distributed under a BSD license.                      ***/
/***   See LICENSE.txt for details.                                       ***/
/***                                                                      ***/
/**************************************************************************+*/

#ifndef FILE_WZ4FRLIB_PHYSX_HPP
#define FILE_WZ4FRLIB_PHYSX_HPP

#include "wz4frlib/wz4_demo2_ops.hpp"
#include "wz4frlib/wz4_physx_ops.hpp"

/****************************************************************************/
/****************************************************************************/

void PhysXInitEngine();

/****************************************************************************/
/****************************************************************************/

// A template tree scene
// Transform and Render objects in a graph

template <typename  T, class T2>
class WpxGenericGraph : public T2
{
public:
  sArray<sMatrix34CM> Matrices;       // matrices list
  sArray<T *> Childs;                 // childs objects

  ~WpxGenericGraph();

  virtual void Render(Wz4RenderContext &ctx, sMatrix34 &mat);   // render
  virtual void Transform(const sMatrix34 & mat);                // build list of model matrices with GRAPH!

  void ClearMatricesR();                                        // clear matrices
  void RenderChilds(Wz4RenderContext &ctx, sMatrix34 &mat);     // recurse to childs
  void TransformChilds(const sMatrix34 & mat);                  // recurse to childs
};

template <typename  T, class T2>
WpxGenericGraph<T, T2>::~WpxGenericGraph()
{
  sReleaseAll(Childs);
}

template <typename  T, class T2>
void WpxGenericGraph<T, T2>::ClearMatricesR()
{
  T *c;
  Matrices.Clear();
  sFORALL(Childs, c)
    c->ClearMatricesR();
}

template <typename  T, class T2>
void WpxGenericGraph<T, T2>::Transform(const sMatrix34 &mat)
{
  TransformChilds(mat);
}

template <typename  T, class T2>
void WpxGenericGraph<T, T2>::TransformChilds(const sMatrix34 &mat)
{
  T *c;

  Matrices.AddTail(sMatrix34CM(mat));

  sFORALL(Childs, c)
    c->Transform(mat);
}

template <typename  T, class T2>
void WpxGenericGraph<T, T2>::Render(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  RenderChilds(ctx,mat);
}

template <typename  T, class T2>
void WpxGenericGraph<T, T2>::RenderChilds(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  // recurse to childs
  T *c;
  sFORALL(Childs, c)
    c->Render(ctx,mat);
}

/****************************************************************************/
/****************************************************************************/

// WpxColliderBase is the base type for all colliders operators
// WpxColliderBase inherited classes are used to preview a graph of colliders

class WpxColliderBase : public WpxGenericGraph<WpxColliderBase, wObject>
{
public:
  WpxColliderBase();
  void AddCollidersChilds(wCommand *cmd);    // add childs
};

/****************************************************************************/
/****************************************************************************/

class WpxCollider : public WpxColliderBase
{
private:
  Wz4Mesh * MeshCollider;   // collider mesh, used to preview collider geometry
  Wz4Mesh * MeshInput;      // ptr to optional mesh used to compute collider geometry (hullmesh or mesh collider type)

public:
  WpxColliderParaCollider ParaBase, Para;

  WpxCollider();
  ~WpxCollider();
  void Transform(const sMatrix34 & mat);
  void Render(Wz4RenderContext &ctx, sMatrix34 &mat);

  void CreateGeometry(Wz4Mesh * input);
};

/****************************************************************************/

class WpxColliderAdd : public WpxColliderBase
{
public:
  WpxColliderAddParaColliderAdd ParaBase, Para;
};

/****************************************************************************/

class WpxColliderTransform : public WpxColliderBase
{
public:
  WpxColliderTransformParaColliderTransform ParaBase, Para;

  void Transform(const sMatrix34 & mat);
};

/****************************************************************************/

class WpxColliderMul : public WpxColliderBase
{
private:
  sMatrix34 MulMatrix;

public:
  WpxColliderMulParaColliderMul ParaBase, Para;

  void Transform(const sMatrix34 & mat);
};

/****************************************************************************/
/****************************************************************************/

// WpxActorBase is the base type for all actors operators
// WpxActorBase inherited classes are used to preview a graph of actors + associated colliders graph

class WpxActorBase : public WpxGenericGraph<WpxActorBase, Wz4Render>
{
public:
  WpxActorBase();
  void AddActorsChilds(wCommand *cmd);    // add childs
};

/****************************************************************************/
/****************************************************************************/

class WpxRigidBody : public WpxActorBase
{
public:
  WpxColliderBase * RootCollider;    // associated colliders geometries, root collider in collider graph

  WpxRigidBodyParaRigidBody ParaBase, Para;

  WpxRigidBody();
  ~WpxRigidBody();
  void Transform(const sMatrix34 & mat);
  void Render(Wz4RenderContext &ctx, sMatrix34 &mat);

  void AddRootCollider(WpxColliderBase * col);
};

/****************************************************************************/

class WpxRigidBodyAdd : public WpxActorBase
{
public:
  WpxRigidBodyAddParaRigidBodyAdd ParaBase, Para;
};

/****************************************************************************/

class WpxRigidBodyTransform : public WpxActorBase
{
public:
  WpxRigidBodyTransformParaRigidBodyTransform ParaBase, Para;

  void Transform(const sMatrix34 & mat);
};

/****************************************************************************/

class WpxRigidBodyMul : public WpxActorBase
{
public:
  WpxRigidBodyMulParaRigidBodyMul ParaBase, Para;

  void Transform(const sMatrix34 & mat);
};

/****************************************************************************/
/****************************************************************************/

/****************************************************************************/
// next Wz4RenderNodes, are nodes associated with actors operators,
// they are computed in the Wz4Render graph process at each render loop,
// they are used for :
// - render real RenderNode binded with physx
// - simulate and process manually physx features like kinematics, add forces, etc...
/****************************************************************************/

class WpxRigidBodyNode : public  Wz4RenderNode
{
public:
  WpxRigidBodyParaRigidBody ParaBase, Para;
};

/****************************************************************************/
/****************************************************************************/

class RNPhysx : public Wz4RenderNode
{
private:

public:
  Wz4RenderParaPhysx ParaBase, Para;
  Wz4RenderAnimPhysx Anim;
};

#endif FILE_WZ4FRLIB_PHYSX_HPP
