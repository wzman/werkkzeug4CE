/*+**************************************************************************/
/***                                                                      ***/
/***   This file is distributed under a BSD license.                      ***/
/***   See LICENSE.txt for details.                                       ***/
/***                                                                      ***/
/**************************************************************************+*/

#include "wz4_physx.hpp"
#include "wz4frlib/wz4_demo2nodes.hpp"

/****************************************************************************/
/****************************************************************************/

void PhysXInitEngine()
{
}

/****************************************************************************/
/****************************************************************************/

enum E_GEOMETRY_TYPE
{
  EGT_CUBE = 0,
  EGT_SPHERE,
  EGT_PLANE,      // note : physx don't support plane colliding with another plane
  EGT_HULL,       // note : convex hull mesh is limited to 256 polygons
  EGT_MESH,       // note : physx don't support mesh colliding with another mesh
  EGT_NONE
};

/****************************************************************************/
/****************************************************************************/

WpxColliderBase::WpxColliderBase()
{
  Type = WpxColliderBaseType;
}

void WpxColliderBase::AddCollidersChilds(wCommand *cmd)
{
  for (sInt i = 0; i<cmd->InputCount; i++)
  {
    WpxColliderBase * in = cmd->GetInput<WpxColliderBase *>(i);
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

/****************************************************************************/

WpxCollider::WpxCollider()
{
  MeshCollider = 0;
  MeshInput = 0;
}

WpxCollider::~WpxCollider()
{
  MeshCollider->Release();
  MeshInput->Release();
}

void WpxCollider::Render(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  if (MeshCollider)
  {
    // render mesh geometry (instance mode)
    sMatrix34CM * cm = &Matrices[0];
    MeshCollider->RenderInst(sRF_TARGET_MAIN, 0, Matrices.GetCount(), cm, 0);
    MeshCollider->RenderInst(sRF_TARGET_WIRE, 0, Matrices.GetCount(), cm, 0);
  }
}

void WpxCollider::Transform(const sMatrix34 & mat)
{
  sMatrix34 initialMat;
  sSRT srt;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(initialMat);

  TransformChilds(initialMat*mat);
}

void WpxCollider::CreateGeometry(Wz4Mesh * input)
{
  // if mesh input, add ref for clean delete
  if (input)
  {
    MeshInput = input;
    input->AddRef();
  }

  sVector31 ShapeSize(1.0f);
  MeshCollider = new Wz4Mesh();
  MeshCollider->MakeSphere(12, 12);

  // center shape, except for hull or mesh
  if (Para.GeometryType != EGT_HULL && Para.GeometryType != EGT_MESH)
  {
    Wz4MeshVertex *mv;
    sAABBox bounds;
    sFORALL(MeshCollider->Vertices, mv)
      bounds.Add(mv->Pos);
    sVector30 d = (sVector30(bounds.Max) + sVector30(bounds.Min))*-0.5f;
    sFORALL(MeshCollider->Vertices, mv)
      mv->Pos += d;
    MeshCollider->Flush();
  }

  // set a material
  Wz4MeshCluster * cluster = new  Wz4MeshCluster();
  MeshCollider->Clusters.AddTail(cluster);
  SimpleMtrl *mtrl = new SimpleMtrl;
  mtrl->SetMtrl(sMTRL_ZOFF | sMTRL_CULLOFF | sMTRL_MSK_GREEN | sMTRL_MSK_BLUE, 0);
  mtrl->Prepare();
  MeshCollider->Clusters[0]->Mtrl = mtrl;
}

/****************************************************************************/

void WpxColliderTransform::Transform(const sMatrix34 & mat)
{
  sSRT srt;
  sMatrix34 mul;

  srt.Scale = Para.Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  TransformChilds(mul*mat);
}

/****************************************************************************/

void WpxColliderMul::Transform(const sMatrix34 & mat)
{
  sSRT srt;
  sMatrix34 preMat;
  sMatrix34 mulMat;
  sMatrix34 accu;

  srt.Scale = Para.PreScale;
  srt.Rotate = Para.PreRot;
  srt.Translate = Para.PreTrans;
  srt.MakeMatrix(preMat);

  srt.Scale = Para.Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mulMat);

  if (Para.Count>1 && (Para.Flags & 1))
    accu.l = sVector31(sVector30(srt.Translate) * ((Para.Count - 1)*-0.5));

  for (sInt i = 0; i<sMax(1, Para.Count); i++)
  {
    TransformChilds(preMat*accu*mat);
    accu = accu * mulMat;
  }
}

/****************************************************************************/
/****************************************************************************/

WpxActorBase::WpxActorBase()
{
  Type = WpxActorBaseType;
}

void WpxActorBase::AddActorsChilds(wCommand *cmd)
{
  for (sInt i = 0; i<cmd->InputCount; i++)
  {
    WpxActorBase * in = cmd->GetInput<WpxActorBase *>(i);
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

/****************************************************************************/

WpxRigidBody::WpxRigidBody()
{
  RootCollider = 0;
}

WpxRigidBody::~WpxRigidBody()
{
  RootCollider->Release();
}

void WpxRigidBody::AddRootCollider(WpxColliderBase * col)
{
  RootCollider = col;
  col->AddRef();
}

void WpxRigidBody::Transform(const sMatrix34 & mat)
{
  sSRT srt;
  sMatrix34 mul;

  srt.Scale = Para.Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  TransformChilds(mul*mat);
}


void WpxRigidBody::Render(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  // get actor initial matrix
  sMatrix34 mat0 = sMatrix34(Matrices[0]);

  // prepare and transform RootNode with actor initial matrix
  RootNode->Prepare(&ctx);
  RootNode->ClearMatricesR();
  RootNode->ClearRecFlagsR();
  RootNode->Transform(&ctx, mat0);
  ctx.ClearRecFlags(RootNode);

  // transform root collider with actor initial matrix
  RootCollider->ClearMatricesR();
  RootCollider->Transform(mat0);

  // for each matrix in graph, render actor + its colliders
  sMatrix34CM * m;
  sFORALL(Matrices, m)
  {
    // render actor
    RootNode->Render(&ctx);

    // render colliders graph from root collider
    RootCollider->Render(ctx, sMatrix34(*m));
  }
}
