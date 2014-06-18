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

class WpxColliderBase : public wObject
{
public:
  sArray<sMatrix34CM> Matrices;       // matrices list
  sArray<WpxColliderBase *> Childs;   // childs tree

  WpxColliderBase();
  ~WpxColliderBase();

  virtual void Render(sFrustum &fr, sMatrix34 * mat); // render collider geometry
  virtual void Transform(const sMatrix34 & mat);      // build list of model matrices with GRAPH!

  void ClearMatricesR();                              // clear matrices
  void RenderChilds(sFrustum &fr, sMatrix34 * mat);   // recurse to childs
  void TransformChilds(const sMatrix34 & mat);        // recurse to childs
  void AddChilds(wCommand *cmd);                      // add childs
};

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
  void Render(sFrustum &fr, sMatrix34 * mat);

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
