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

PxFoundation * gFoundation;   // global physx foundation pointer
PxPhysics * gPhysicsSDK;      // global physx engine pointer
PxCooking * gCooking;         // global physx cooking pointer

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

/****************************************************************************/
/****************************************************************************/

void PhysXInitEngine()
{
  // create foundation object with default error and allocator callbacks.
  gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION,gDefaultAllocatorCallback,gDefaultErrorCallback);
  if(!gFoundation)
  {
    sLogF(L"PhysX",L"PhysXInitEngine - PxCreateFoundation failed!\n");
    return;
  }

  // create Physics object with the created foundation and with a 'default' scale tolerance.
  gPhysicsSDK = PxCreatePhysics(PX_PHYSICS_VERSION,*gFoundation,PxTolerancesScale());
  if(!gPhysicsSDK)
  {
    sLogF(L"PhysX",L"PhysXInitEngine - PxCreatePhysics failed!\n");
    return;
  }

  // create cooking object
  gCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams());
  if (!gCooking)
  {
    sLogF(L"PhysX",L"PhysXInitEngine - PxCreateCooking failed!\n");
    return;
  }

  // PVD - init the Physx visual debugger
#ifdef COMPIL_WITH_PVD
  // check if PvdConnection manager is available on this platform
  if (gPhysicsSDK->getPvdConnectionManager() == NULL)
    return;

  // setup connection parameters
  const char*     pvd_host_ip = "127.0.0.1";  // IP of the PC which is running PVD
  int             port = 5425;         // TCP port to connect to, where PVD is listening
  unsigned int    timeout = 100;          // timeout in milliseconds to wait for PVD to respond,
  // consoles and remote PCs need a higher timeout.
  PxVisualDebuggerConnectionFlags connectionFlags = PxVisualDebuggerExt::getAllConnectionFlags();

  // and now try to connect
  PVD::PvdConnection* theConnection = PxVisualDebuggerExt::createConnection(gPhysicsSDK->getPvdConnectionManager(),
    pvd_host_ip, port, timeout, connectionFlags);

  // remember to release the connection by manual in the end
  if (theConnection)
    theConnection->release();
#endif
}

/****************************************************************************/

sINLINE void sMatrix34ToPxMat44(const sMatrix34 &wzMat, PxMat44 &pxMat)
{
  PxVec3 pxv0(wzMat.i.x, wzMat.i.y, wzMat.i.z);
  PxVec3 pxv1(wzMat.j.x, wzMat.j.y, wzMat.j.z);
  PxVec3 pxv2(wzMat.k.x, wzMat.k.y, wzMat.k.z);
  PxVec3 pxv3(wzMat.l.x, wzMat.l.y, wzMat.l.z);

  pxMat = PxMat44(pxv0, pxv1, pxv2, pxv3);
}

/****************************************************************************/

sINLINE void PxMat44TosMatrix34(const PxMat44 &pxMat, sMatrix34 &wzMat)
{
  wzMat.i.x = pxMat.column0.x;
  wzMat.i.y = pxMat.column0.y;
  wzMat.i.z = pxMat.column0.z;

  wzMat.j.x = pxMat.column1.x;
  wzMat.j.y = pxMat.column1.y;
  wzMat.j.z = pxMat.column1.z;

  wzMat.k.x = pxMat.column2.x;
  wzMat.k.y = pxMat.column2.y;
  wzMat.k.z = pxMat.column2.z;

  wzMat.l.x = pxMat.column3.x;
  wzMat.l.y = pxMat.column3.y;
  wzMat.l.z = pxMat.column3.z;
}

/****************************************************************************/

void PxConvexMeshToWz4Mesh(const PxConvexMesh * pxConvexMesh, Wz4Mesh * wz4Mesh)
{
  sVERIFY(wz4Mesh);

  PxU32 nbVerts = pxConvexMesh->getNbVertices();
  const PxVec3* convexVerts = pxConvexMesh->getVertices();
  const PxU8* indexBuffer = pxConvexMesh->getIndexBuffer();
  PxU32 nbPolygons = pxConvexMesh->getNbPolygons();

  PxU32 totalNbTris = 0;
  PxU32 totalNbVerts = 0;
  for(PxU32 i=0;i<nbPolygons;i++)
  {
    PxHullPolygon data;
    bool status = pxConvexMesh->getPolygonData(i, data);
    PX_ASSERT(status);
    totalNbVerts += data.mNbVerts;
    totalNbTris += data.mNbVerts - 2;
  }

  wz4Mesh->AddDefaultCluster();

  PxU32 offset = 0;
  for(PxU32 i=0;i<nbPolygons;i++)
  {
    PxHullPolygon face;
    bool status = pxConvexMesh->getPolygonData(i, face);
    PX_ASSERT(status);

    const PxU8* faceIndices = indexBuffer + face.mIndexBase;
    for(PxU32 j=0;j<face.mNbVerts;j++)
    {
      sVector31 v;
      v.x = convexVerts[faceIndices[j]].x;
      v.y = convexVerts[faceIndices[j]].y;
      v.z = convexVerts[faceIndices[j]].z;

      Wz4MeshVertex mv;
      mv.Init();
      mv.Pos.x = v.x;
      mv.Pos.y = v.y;
      mv.Pos.z = v.z;

      wz4Mesh->Vertices.AddTail(mv);
    }

    for(PxU32 j=1;j<face.mNbVerts;j++)
    {
      Wz4MeshFace mf;
      mf.Init(3);
      mf.Vertex[0] = PxU16(offset);
      mf.Vertex[1] = PxU16(offset+j);
      mf.Vertex[2] = PxU16(offset+j-1);
      wz4Mesh->Faces.AddTail(mf);
    }

    offset += face.mNbVerts;
  }
}

/****************************************************************************/

PxConvexMesh * MakePxConvexHull(Wz4Mesh * srcMesh)
{
  sArray<PxVec3> convexVertices;

  // copy all wz4mesh vertices into convexVertices array
  Wz4MeshVertex * mv;
  sFORALL(srcMesh->Vertices, mv)
  {
    PxVec3 pxv;
    pxv.x = mv->Pos.x;
    pxv.y = mv->Pos.y;
    pxv.z = mv->Pos.z;

    convexVertices.AddTail(pxv);
  }

  // generate hull from convex Description
  PxConvexMeshDesc convexDesc;
  convexDesc.points.count     = srcMesh->Vertices.GetCount();
  convexDesc.points.stride    = sizeof(PxVec3);
  convexDesc.points.data      = convexVertices.GetData();
  convexDesc.flags            = PxConvexFlag::eCOMPUTE_CONVEX;

  PxDefaultMemoryOutputStream os;
  if(!gCooking->cookConvexMesh(convexDesc, os))
  {
    // failed to create convex hull mesh
    return 0;
  }

  PxU8 * data = os.getData();
  PxDefaultMemoryInputData * input = new PxDefaultMemoryInputData(data, os.getSize());
  PxConvexMesh * convexMesh = gPhysicsSDK->createConvexMesh(*input);
  delete input;

  return convexMesh;
}

/****************************************************************************/

PxTriangleMesh * MakePxMesh(Wz4Mesh * srcMesh)
{
  sArray<PxVec3> meshVertices;

  // copy all wz4mesh vertices into convexVertices array
  Wz4MeshVertex * mv;
  sFORALL(srcMesh->Vertices, mv)
  {
    PxVec3 pxv;
    pxv.x = mv->Pos.x;
    pxv.y = mv->Pos.y;
    pxv.z = mv->Pos.z;

    meshVertices.AddTail(pxv);
  }

  sArray<PxU32> meshFaces;
  Wz4MeshFace * mf;
  sFORALL(srcMesh->Faces, mf)
  {
    meshFaces.AddTail(mf->Vertex[0]);
    meshFaces.AddTail(mf->Vertex[1]);
    meshFaces.AddTail(mf->Vertex[2]);
  }

  PxTriangleMeshDesc meshDesc;
  meshDesc.points.count           = meshVertices.GetCount();
  meshDesc.points.stride          = sizeof(PxVec3);
  meshDesc.points.data            = meshVertices.GetData();

  meshDesc.triangles.count        = meshFaces.GetCount()/3;
  meshDesc.triangles.stride       = 3*sizeof(PxU32);
  meshDesc.triangles.data         = meshFaces.GetData();


  PxDefaultMemoryOutputStream os;
  if(!gCooking->cookTriangleMesh(meshDesc, os))
  {
    // failed to create px triangulate mesh
    return 0;
  }

  PxU8 * data = os.getData();
  PxDefaultMemoryInputData * input = new PxDefaultMemoryInputData(data, os.getSize());
  PxTriangleMesh * triMesh = gPhysicsSDK->createTriangleMesh(*input);
  delete input;

  return triMesh;
}

/****************************************************************************/

void PxTriMeshToWz4Mesh(const PxTriangleMesh * pxConvexMesh, Wz4Mesh * wz4Mesh)
{
  sVERIFY(wz4Mesh);
  wz4Mesh->AddDefaultCluster();

  const PxU32 nbVerts = pxConvexMesh->getNbVertices();
  const PxVec3* verts = pxConvexMesh->getVertices();
  const PxU32 nbTris = pxConvexMesh->getNbTriangles();
  const void* tris = pxConvexMesh->getTriangles();

  for(PxU32 i=0; i<nbVerts; i++)
  {
    Wz4MeshVertex mv;
    mv.Init();
    mv.Pos.x = verts->x;
    mv.Pos.y = verts->y;
    mv.Pos.z = verts->z;
    wz4Mesh->Vertices.AddTail(mv);
    verts++;
  }

  const PxU16* src = (const PxU16*)tris;
  for(PxU32 i=0;i<nbTris;i++)
  {
    Wz4MeshFace mf;
    mf.Init(3);
    mf.Vertex[0] = src[i*3+0];
    mf.Vertex[1] = src[i*3+2];
    mf.Vertex[2] = src[i*3+1];
    wz4Mesh->Faces.AddTail(mf);
  }
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

void WpxColliderBase::GetDensity(sArray<PxReal> * densities)
{
  GetDensityChilds(densities);
}

void WpxColliderBase::GetDensityChilds(sArray<PxReal> * densities)
{
  WpxColliderBase * c;
  sFORALL(Childs, c)
  {
    c->GetDensity(densities);
  }
}

/****************************************************************************/

WpxCollider::WpxCollider()
{
  MeshCollider = 0;
  MeshInput = 0;
  ConvexMesh = 0;
  TriMesh = 0;
}

WpxCollider::~WpxCollider()
{
  MeshCollider->Release();
  MeshInput->Release();
  if(ConvexMesh)
    ConvexMesh->release();
  if(TriMesh)
    TriMesh->release();
}


void WpxCollider::GetDensity(sArray<PxReal> * densities)
{
  densities->AddTail(Para.Density);
}

void WpxCollider::Render(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  if (MeshCollider)
  {
    // render mesh geometry (instance mode)
    sMatrix34CM * cm = &Matrices[0];

    if( (ctx.RenderFlags&wRF_RenderWire) == 0)
      MeshCollider->RenderInst(sRF_TARGET_MAIN, 0, Matrices.GetCount(), cm, 0);

    MeshCollider->RenderInst(sRF_TARGET_WIRE, 0, Matrices.GetCount(), cm, 0);
  }
}

void WpxCollider::Transform(const sMatrix34 & mat, PxRigidActor * ptr)
{
  sMatrix34 mul;
  sSRT srt;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  if (!ptr)
    TransformChilds(mul*mat, 0);
  else
  {
    // ptr is not null : create physx collider for this actor
    CreatePhysxCollider(ptr, sMatrix34(mat));
  }
}

void WpxCollider::CreatePhysxCollider(PxRigidActor * actor, sMatrix34 & mat)
{
  // create a physx material
  PxMaterial* material = 0;
  material = gPhysicsSDK->createMaterial(Para.StaticFriction, Para.DynamicFriction, Para.Restitution);
  sVERIFY(material != 0);

  // create geometry according type
  PxGeometry * geometry = 0;
  switch (Para.GeometryType)
  {
    case EGT_CUBE:
    {
      PxVec3 dimensions(Para.Dimension.x/2, Para.Dimension.y/2, Para.Dimension.z/2);
      geometry = new PxBoxGeometry(dimensions);
    }
    break;

    case EGT_SPHERE:
    {
      geometry = new PxSphereGeometry(Para.Radius/2);
    }
    break;

    case EGT_PLANE:
    {
      geometry = new PxPlaneGeometry();
    }
    break;

    case EGT_HULL:
    {
      geometry = new PxConvexMeshGeometry(ConvexMesh);
    }
    break;

    case EGT_MESH:
    {
      geometry = new PxTriangleMeshGeometry(TriMesh);
    }
    break;
  }
  sVERIFY(geometry != 0);

  // compute matrix transformation
  sMatrix34 wzMat34;
  sSRT srt;
  srt.Scale = sVector31(1.0f);
  srt.Rotate = Para.Rot;
  if (Para.GeometryType == EGT_PLANE)
    srt.Rotate.z += 0.25;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(wzMat34);

  // multiply by result graph matrices
  wzMat34 *= mat;

  // apply transformation to collider
  PxMat44 pxMat;
  sMatrix34ToPxMat44(wzMat34, pxMat);
  PxTransform transform(pxMat);

  // create shape for actor
  actor->createShape(*geometry, *material, transform);

  // delete no more used stuff
  delete geometry;
  material->release();
  if (ConvexMesh)
    ConvexMesh->release();
  if (TriMesh)
    TriMesh->release();
}

sBool WpxCollider::CreateGeometry(Wz4Mesh * input)
{
  // if mesh input, add ref for clean delete
  if (input)
  {
    MeshInput = input;
    input->AddRef();
  }

  sVector31 ShapeSize(1.0f);
  MeshCollider = new Wz4Mesh();

  switch (Para.GeometryType)
  {
  case EGT_CUBE:
    MeshCollider->MakeCube(1, 1, 1);
    ShapeSize = Para.Dimension;
    break;

  case EGT_SPHERE:
    MeshCollider->MakeSphere(12, 12);
    ShapeSize = sVector31(Para.Radius);
    break;

  case EGT_PLANE:
    MeshCollider->MakeGrid(16, 16);
    ShapeSize = Para.Dimension;
    break;

  case EGT_HULL:
    ConvexMesh = MakePxConvexHull(input);
    if (ConvexMesh == 0)
    {
      // failed to create PxConvexMesh
      delete MeshCollider;
      MeshCollider = 0;
      return sFALSE;
    }
    else
    {
      PxConvexMeshToWz4Mesh(ConvexMesh, MeshCollider);
    }
    break;

  case EGT_MESH:
    TriMesh = MakePxMesh(input);
    if (TriMesh == 0)
    {
      // failed to create PxConvexMesh
      delete MeshCollider;
      MeshCollider = 0;
      return sFALSE;
    }
    else
    {
      PxTriMeshToWz4Mesh(TriMesh, MeshCollider);
    }
    break;
  }

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

  // apply transformation matrix to mesh
  sMatrix34 mul;
  sSRT srt;
  srt.Scale = ShapeSize;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);
  MeshCollider->Transform(mul);

  // set a material
  Wz4MeshCluster * cluster = new  Wz4MeshCluster();
  MeshCollider->Clusters.AddTail(cluster);
  SimpleMtrl *mtrl = new SimpleMtrl;
  mtrl->SetMtrl(sMTRL_ZOFF | sMTRL_CULLOFF | sMTRL_MSK_GREEN | sMTRL_MSK_BLUE, 0);
  mtrl->Prepare();
  MeshCollider->Clusters[0]->Mtrl = mtrl;

  return sTRUE;
}

/****************************************************************************/

void WpxColliderTransform::Transform(const sMatrix34 & mat, PxRigidActor * ptr)
{
  sSRT srt;
  sMatrix34 mul;

  srt.Scale = Para.Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  TransformChilds(mul*mat, ptr);
}

/****************************************************************************/

void WpxColliderMul::Transform(const sMatrix34 & mat, PxRigidActor * ptr)
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
    TransformChilds(preMat*accu*mat, ptr);
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
      if (in->IsType(WpxActorBaseType))
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
  PhysxReset();
}

void WpxRigidBody::AddRootCollider(WpxColliderBase * col)
{
  RootCollider = col;
  col->AddRef();
}

void WpxRigidBody::PhysxReset()
{
  // delete all actors
  sActor * a;
  sFORALL(AllActors, a)
  {
    delete a->matrix;
    delete a;
  }

  AllActors.Clear();
}

void WpxRigidBody::PhysxBuildActor(const sMatrix34 & mat, PxScene * scene)
{
  // ptr for local use
  PxRigidDynamic * rigidDynamic = 0;
  PxRigidStatic * rigidStatic = 0;

  // create new actor
  sActor * actor = new sActor;

  // create physx rigid body
  if (Para.ActorType == 0)
  {
    // create static actor
    actor->actor = gPhysicsSDK->createRigidStatic(PxTransform::createIdentity());

    // set static ptr
    rigidStatic = static_cast<PxRigidStatic*>(actor->actor);
  }
  else
  {
    // dynamic actor
    actor->actor = gPhysicsSDK->createRigidDynamic(PxTransform::createIdentity());

    // set dynamic ptr
    rigidDynamic = static_cast<PxRigidDynamic*>(actor->actor);

    // kinematic ?
    if (Para.ActorType == 1 && Para.DynamicType == 1)
      rigidDynamic->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);
  }

  // process collider graph transformations to create colliders for this actors
  sMatrix34 initMat;
  initMat.Init();
  RootCollider->Transform(initMat, actor->actor);

  // compute actor pose
  PxMat44 pxMat;
  sMatrix34ToPxMat44(mat, pxMat);
  PxTransform pose(pxMat);

  // set actor pose
  actor->actor->setGlobalPose(pose);

  // compute mass and inertia
  if (Para.ActorType == 1)
  {
    sArray<PxReal> Densities;
    RootCollider->GetDensity(&Densities);
    PxReal * sd = Densities.GetData();
    PxRigidBodyExt::updateMassAndInertia(*rigidDynamic, sd, Densities.GetCount());
  }

  // add actor to physx scene
  scene->addActor(*actor->actor);

  // copy matrix (used by kinematics)
  actor->matrix = new sMatrix34(mat);

  // add actor to list
  AllActors.AddTail(actor);

  // get WpxRigidBodyNodeDynamic from rootnode
  /*WpxRigidBodyNodeDynamic * rigidNode = static_cast<WpxRigidBodyNodeDynamic *>(RootNode);
  if (rigidNode)
  {
    // set actor list ptr
    rigidNode->AllActorsPtr = &AllActors;
  }*/

  // get WpxRigidBodyNodeDynamic from rootnode
  WpxRigidBodyNodeActor * rigidNode = static_cast<WpxRigidBodyNodeActor *>(RootNode);
  if (rigidNode)
  {
    // set actor list ptr
    rigidNode->AllActorsPtr = &AllActors;
  }
}

void WpxRigidBody::Transform(const sMatrix34 & mat, PxScene * ptr)
{
  sSRT srt;
  sMatrix34 mul, mulmat;

  srt.Scale = Para.Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  mulmat = mul*mat;

  if (!ptr)
  {
    // ptr is null : transform is calling for WpxRigidBody Rendering

    // a WpxRigidBody has no WpxRigidBody childs, so no need to transform Childs
    // but need at least one matrix when building physx actor
    Matrices.AddTail(sMatrix34CM(mulmat));

    // instead of WpxRigidBody it has a RootCollider and a RootNode, so transform them
    RootCollider->Transform(mulmat, 0);
    RootNode->Transform(0, mulmat);

  }
  else
  {
    // ptr not null : transform is calling from Physx init to build physx objects
    PhysxBuildActor(mulmat, ptr);
  }
}

void WpxRigidBody::ClearMatricesR()
{
  Matrices.Clear();

  RootCollider->ClearMatricesR();
  RootNode->ClearMatricesR();
}

void WpxRigidBody::Render(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  // RenderNode render
  ctx.ClearRecFlags(RootNode);
  RootNode->Render(&ctx);

  // colliders render
  RootCollider->Render(ctx, mat);
}

/****************************************************************************/

void WpxRigidBodyTransform::Transform(const sMatrix34 & mat, PxScene * ptr)
{
  sSRT srt;
  sMatrix34 mul;

  srt.Scale = Para.Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  TransformChilds(mul*mat, ptr);
}

/****************************************************************************/

void WpxRigidBodyMul::Transform(const sMatrix34 & mat, PxScene * ptr)
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
    TransformChilds(preMat*accu*mat, ptr);
    accu = accu * mulMat;
  }
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

void WpxRigidBodyNodeActor::Transform(Wz4RenderContext *ctx, const sMatrix34 & mat)
{
  if (ctx)
  {
    // transformed by Wz4RenderNode process at each loop
    // transform associated scene node with physx

    sActor * a;
    PxTransform pT;
    sMatrix34 mmat;
    sMatrix34CM matRes;
    sFORALL(*AllActorsPtr, a)
    {
      pT = a->actor->getGlobalPose();
      PxMat44TosMatrix34(pT, mmat);
      matRes = mmat*mat;
      Childs[0]->Matrices.AddTail(matRes);
      //Childs[0]->Matrices.AddTail(sMatrix34CM(mmat*mat));
      //TransformChilds(ctx, mmat);
    }
  }
  else
  {
    // transformed by WpxRigidBody (preview actors positions without physx)
    // transform associated scene node without physx

    TransformChilds(ctx, mat);
  }
}

/****************************************************************************/

WpxRigidBodyNodeKinematic::WpxRigidBodyNodeKinematic()
{
  Anim.Init(Wz4RenderType->Script);
}

void WpxRigidBodyNodeKinematic::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);

  PxMat44 pxMat;
  sActor * a;
  sMatrix34 wzMat34;
  sSRT srt;
  srt.Scale = sVector31(1.0f);
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(wzMat34);

  PxRigidDynamic * r;
  sFORALL(*AllActorsPtr, a)
  {
    sMatrix34ToPxMat44(wzMat34**a->matrix, pxMat);
    PxTransform transform(pxMat);

    r = static_cast<PxRigidDynamic*>(a->actor);
    r->setKinematicTarget(transform);
  }

  SimulateChilds(ctx);
}

/****************************************************************************/
/****************************************************************************/

RNPhysx::RNPhysx()
{
  Scene = 0;

  PreviousTimeLine = 0.0f;
  Executed = sFALSE;

  Anim.Init(Wz4RenderType->Script);
}

RNPhysx::~RNPhysx()
{
  // delete physx scene
  if (Scene)
    Scene->release();
}

sBool RNPhysx::Init(wCommand *cmd)
{
  // create a new physx scene
  Scene = CreateScene();
  if (!Scene)
    return sFALSE;

  // create first matrix
  sMatrix34 mat;
  mat.Init();

  // for each childs
  for (sInt i=0; i<cmd->InputCount; i++)
  {
    WpxActorBase *in = cmd->GetInput<WpxActorBase *>(i);
    if(in)
    {
      // delete physx objects if it already exists
      in->PhysxReset();

      // process graph transformation, create physx objects and add them to physx scene
      in->Transform(mat, Scene);
    }
  }

  return sTRUE;
}

PxScene * RNPhysx::CreateScene()
{
  // Create the scene
  PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(Para.Gravity.x, Para.Gravity.y, Para.Gravity.z);
  if (!sceneDesc.cpuDispatcher)
  {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(Para.NumThreads);

    if (!mCpuDispatcher)
    {
      sLogF(L"PhysX", L"CreateScene - PxDefaultCpuDispatcherCreate failed!\n");
      return 0;
    }

    sceneDesc.cpuDispatcher = mCpuDispatcher;
  }

  if (!sceneDesc.filterShader)
    sceneDesc.filterShader = gDefaultFilterShader;


  PxScene * scene = gPhysicsSDK->createScene(sceneDesc);

  if (!scene)
  {
    sLogF(L"PhysX", L"CreateScene - createScene failed!\n");
    return 0;
  }

  return scene;
}

void RNPhysx::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);

  // pause/restart mechanism (F5/F6 key)
  if (!Doc->IsPlayer)
  {
    // compute elpased wz time for the restart mechanism
    sF32 timeLine = ctx->GetBaseTime();
    sF32 deltaTimeLine = timeLine - PreviousTimeLine;
    PreviousTimeLine = timeLine;

    // timeline is paused, no simulation to run
    if (deltaTimeLine == 0)
      return;

    // Restart mechanism (for F6, loop demo, or clip restart)
    if (Executed && deltaTimeLine < -0.1f)
    {
      // rebuild operator (to restore default values)
      Doc->Change(Op, 1, 1);
      Executed = sFALSE;
      return;
    }
    Executed = sTRUE;
  }

  SimulateChilds(ctx);


  // compute physx
  sF32 stepSize = 1.0f / 60.0f;
  Scene->simulate(stepSize);
  while (!Scene->fetchResults())
  {
    // do something useful
  }

  ViewPrintF(L"Scene actors : dynamic %d, static %d : %d\n",
      Scene->getNbActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC),
      Scene->getNbActors(PxActorTypeSelectionFlag::eRIGID_STATIC));
}
