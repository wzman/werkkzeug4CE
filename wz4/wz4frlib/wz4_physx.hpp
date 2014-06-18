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

template <typename  T>
class WpxGenericGraph : public wObject
{
public:
  sArray<sMatrix34CM> Matrices;       // matrices list
  sArray<T *> Childs;                 // childs tree

  WpxGenericGraph();
  ~WpxGenericGraph();

  virtual void Render(sFrustum &fr);                  // render collider geometry
  virtual void Transform(const sMatrix34 & mat);      // build list of model matrices with GRAPH!

  void ClearMatricesR();                              // clear matrices
  void RenderChilds(sFrustum &fr);                    // recurse to childs
  void TransformChilds(const sMatrix34 & mat);        // recurse to childs
  void AddChilds(wCommand *cmd);                      // add childs
};

template<typename T>
WpxGenericGraph<T>::WpxGenericGraph()
{
  Type = WpxColliderBaseType;
}

template<typename T>
WpxGenericGraph<T>::~WpxGenericGraph()
{
  sReleaseAll(Childs);
}

template<typename T>
void WpxGenericGraph<T>::AddChilds(wCommand *cmd)
{
  for (sInt i = 0; i<cmd->InputCount; i++)
  {
    T * in = cmd->GetInput<T *>(i);
    if (in)
    {
      if (in->IsType(WpxColliderBaseType))
      {
        Childs.AddTail(in);
        in->AddRef();
      }
    }
  }
}

template<typename T>
void WpxGenericGraph<T>::ClearMatricesR()
{
  T *c;
  Matrices.Clear();
  sFORALL(Childs, c)
    c->ClearMatricesR();
}

template<typename T>
void WpxGenericGraph<T>::Transform(const sMatrix34 &mat)
{
  TransformChilds(mat);
}

template<typename T>
void WpxGenericGraph<T>::TransformChilds(const sMatrix34 &mat)
{
  T *c;

  Matrices.AddTail(sMatrix34CM(mat));

  sFORALL(Childs, c)
    c->Transform(mat);
}

template<typename T>
void WpxGenericGraph<T>::Render(sFrustum &fr)
{
  RenderChilds(fr);
}

template<typename T>
void WpxGenericGraph<T>::RenderChilds(sFrustum &fr)
{
  // recurse to childs
  T *c;
  sFORALL(Childs, c)
    c->Render(fr);
}

/****************************************************************************/
/****************************************************************************/

class WpxColliderBase : public WpxGenericGraph<WpxColliderBase> {};

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

#endif FILE_WZ4FRLIB_PHYSX_HPP
