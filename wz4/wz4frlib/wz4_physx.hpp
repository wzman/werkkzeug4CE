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

template <typename  T, class T2>
class WpxGenericGraph : public T2
{
public:
  sArray<sMatrix34CM> Matrices;       // matrices list
  sArray<T *> Childs;                 // childs objects

  ~WpxGenericGraph();

  virtual void Render(sFrustum &fr);                  // render
  virtual void Transform(const sMatrix34 & mat);      // build list of model matrices with GRAPH!

  void ClearMatricesR();                              // clear matrices
  void RenderChilds(sFrustum &fr);                    // recurse to childs
  void TransformChilds(const sMatrix34 & mat);        // recurse to childs
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
void WpxGenericGraph<T, T2>::Render(sFrustum &fr)
{
  RenderChilds(fr);
}

template <typename  T, class T2>
void WpxGenericGraph<T, T2>::RenderChilds(sFrustum &fr)
{
  // recurse to childs
  T *c;
  sFORALL(Childs, c)
    c->Render(fr);
}

/****************************************************************************/
/****************************************************************************/

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
  void Render(sFrustum &fr);

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
  WpxRigidBodyParaRigidBody ParaBase, Para;
};

/****************************************************************************/

class WpxRigidBodyAdd : public WpxActorBase
{
public:
  WpxRigidBodyAddParaRigidBodyAdd ParaBase, Para;
};

/****************************************************************************/

class WpxRigidBodyMul : public WpxActorBase
{
public:
  WpxRigidBodyMulParaRigidBodyMul ParaBase, Para;
};

/****************************************************************************/

class WpxRigidBodyTransform : public WpxActorBase
{
public:
  WpxRigidBodyTransformParaRigidBodyTransform ParaBase, Para;
};

/****************************************************************************/
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
