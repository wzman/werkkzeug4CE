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

// Globals variables and functions

PxFoundation * gFoundation;   // global physx foundation pointer
PxPhysics * gPhysicsSDK;      // global physx engine pointer
PxCooking * gCooking;         // global physx cooking pointer
sInt gCumulatedCount = 0;     // cumulated forces counter for animated rigid bodies

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

template <typename T>sINLINE void PhysxWakeUpRigidDynamics(sArray<sActor *> * actors, T &para);
template <typename T>sINLINE void SimulateRigidDynamics(sArray<sActor *> * actors, T &para);

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

enum E_ACTOR_TYPE
{
  EAT_STATIC = 0,
  EAT_DYNAMIC,
  EAT_KINEMATIC,
  EAT_NONE
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
  if (!ptr)
    TransformChilds(mat, 0);
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

  srt.Scale = sVector31(1.0f);
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

  srt.Scale = sVector31(1.0f);
  srt.Rotate = Para.PreRot;
  srt.Translate = Para.PreTrans;
  srt.MakeMatrix(preMat);

  srt.Scale = sVector31(1.0f);
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

void WpxActorBase::PhysxReset()
{
  PhysxResetChilds();
}

void WpxActorBase::PhysxResetChilds()
{
  WpxActorBase * c;
  sFORALL(Childs, c)
    c->PhysxReset();
}

void WpxActorBase::PhysxWakeUp()
{
  PhysxWakeUpChilds();
}

void WpxActorBase::PhysxWakeUpChilds()
{
  WpxActorBase * c;
  sFORALL(Childs, c)
    c->PhysxWakeUp();
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

void WpxRigidBody::PhysxWakeUp()
{
  PhysxWakeUpRigidDynamics(&AllActors, Para);
}

void WpxRigidBody::PhysxBuildActor(const sMatrix34 & mat, PxScene * scene, sArray<sActor*> &allActors)
{
  // ptr for local use
  PxRigidDynamic * rigidDynamic = 0;
  PxRigidStatic * rigidStatic = 0;

  // create new actor
  sActor * actor = new sActor;

  // create new actor matrix
  actor->matrix = new sMatrix34(mat);

  // create physx rigid body
  if (Para.ActorType == EAT_STATIC)
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
    if (Para.ActorType == EAT_KINEMATIC)
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

  // set physx properties for dynamics
  if (Para.ActorType == EAT_DYNAMIC)
  {
    // auto MassAndInertia ?
    if (Para.MassAndInertia == 0)
    {
      // auto compute mass and inertia according all shapes colliders densities
      sArray<PxReal> Densities;
      RootCollider->GetDensity(&Densities);
      PxReal * sd = Densities.GetData();
      PxRigidBodyExt::updateMassAndInertia(*rigidDynamic, sd, Densities.GetCount());
    }
    else
    {
      // manual set of mass and center of mass
      PxReal mass = Para.Mass;
      PxVec3 massLocalPose(Para.CenterOfMass.x, Para.CenterOfMass.y, Para.CenterOfMass.z);
      PxRigidBodyExt::setMassAndUpdateInertia(*rigidDynamic, mass, &massLocalPose);
    }

    // sleep threshold
    rigidDynamic->setSleepThreshold(Para.SleepThreshold);

    // gravity flag
    bool gravityFlag = !Para.Gravity;
    rigidDynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, gravityFlag);

    // damping
    PxReal linDamp = Para.LinearDamping;
    rigidDynamic->setLinearDamping(linDamp);
    PxReal angDamp = Para.AngularDamping;
    rigidDynamic->setAngularDamping(angDamp);

    // force
    if (Para.ForceMode != 4)
    {
      PxVec3 force(Para.Force.x, Para.Force.y, Para.Force.z);
      PxForceMode::Enum forceMode = (PxForceMode::Enum)Para.ForceMode;
      rigidDynamic->addForce(force, forceMode);
    }

    // torque
    if (Para.TorqueMode != 4)
    {
      PxVec3 torque(Para.Torque.x, Para.Torque.y, Para.Torque.z);
      PxForceMode::Enum torqueMode = (PxForceMode::Enum)Para.TorqueMode;
      rigidDynamic->addTorque(torque, torqueMode);
    }

    // put to sleep at init
    rigidDynamic->putToSleep();

  }

  // add actor to physx scene
  scene->addActor(*actor->actor);

  // add actor to list
  allActors.AddTail(actor);
}

void WpxRigidBody::Transform(const sMatrix34 & mat, PxScene * ptr)
{
  sSRT srt;
  sMatrix34 mul, mulmat;

  srt.Scale = sVector31(1.0f);
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

    // build physx actors
    PhysxBuildActor(mulmat, ptr, AllActors);

    // kinematics need this matrix as initial pose
    if (Para.ActorType == EAT_KINEMATIC)
      *AllActors.GetTail()->matrix = mat;

    // copy AllActor adress in RootNode
    WpxRigidBodyNodeActor * rigidNode = static_cast<WpxRigidBodyNodeActor *>(RootNode);
    if (rigidNode)
      rigidNode->AllActorsPtr = &AllActors;

    // AllActors is used in RenderNode to get global pose of physx actors
    // its pointer in RootNode must not be null
    sVERIFY(rigidNode!=0);
    sVERIFY(rigidNode->AllActorsPtr!=0);
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

  srt.Scale = sVector31(1.0f);
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

  srt.Scale = sVector31(1.0f);
  srt.Rotate = Para.PreRot;
  srt.Translate = Para.PreTrans;
  srt.MakeMatrix(preMat);

  srt.Scale = sVector31(1.0f);
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

WpxRigidBodyDebris::WpxRigidBodyDebris()
{
  ChunkedMesh = 0;
}

WpxRigidBodyDebris::~WpxRigidBodyDebris()
{
  // delete all elements of sChunkCollider stuct
  sChunkCollider * o;
  sFORALL(ChunksColliders, o)
  {
    o->wCollider->Release();
    o->Mesh->Release();
    delete o;
  }

  WpxRigidBody * b;
  sFORALL(ChunksRigidBodies, b)
  {
    if(b)
      b->Release();
  }

  // delete chunked mesh
  ChunkedMesh->Release();

  // delete physx actors
  sActor * a;
  sFORALL(AllActors, a)
  {
    delete a->matrix;
    delete a;
  }
}

void WpxRigidBodyDebris::Render(Wz4RenderContext &ctx, sMatrix34 &mat)
{
  // render every colliders/chunks

  sChunkCollider * o;
  sFORALL(ChunksColliders, o)
  {
    sVERIFY(o->wCollider->Matrices.GetCount() > 0);

    o->wCollider->Render(ctx, mat);
    o->wCollider->ClearMatricesR();
  }
}

void WpxRigidBodyDebris::PhysxWakeUp()
{
  PhysxWakeUpRigidDynamics(&AllActors, Para);
}

void WpxRigidBodyDebris::PhysxReset()
{
  // delete all rigidbodies
  WpxRigidBody * b;
  sFORALL(ChunksRigidBodies, b)
  {
    if(b)
      b->Release();
  }
  ChunksRigidBodies.Reset();

  // delete all physx actors
  sActor * a;
  sFORALL(AllActors, a)
  {
    delete a->matrix;
    delete a;
  }
  AllActors.Clear();
}

int WpxRigidBodyDebris::GetChunkedMesh(Wz4Render * in)
{
  // get mesh from Wz4RenderNode
  // mesh must be chunked

  RNRenderMesh * rm = static_cast<RNRenderMesh*>(in->RootNode);
  if (rm && rm->Mesh)
  {
    if (rm->Mesh->Chunks.GetCount() > 0)
    {
      ChunkedMesh = rm->Mesh;
      ChunkedMesh->AddRef();
    }
    else
      return 2;  // error, need a chunked mesh
  }
  else
    return 1;   // error, need a mesh input


  // extract all chunks from input mesh
  // and create colliders shapes

  Wz4ChunkPhysics *ch;
  sArray<Wz4MeshVertex> vertices;
  sFORALL(ChunkedMesh->Chunks, ch)
  {
    // get first and last vertices index for current chunk
    sInt start = ch->FirstVert;
    sInt end = -1;
    if (_i + 1 <= ChunkedMesh->Chunks.GetCount() - 1)
      end = ChunkedMesh->Chunks[_i + 1].FirstVert;
    else
      end = ChunkedMesh->Vertices.GetCount();

    // get all chunk's vertices
    for (int i = start; i<end; i++)
      vertices.AddTail(ChunkedMesh->Vertices[i]);

    // create a new mesh with all theses vertices
    Wz4Mesh * m = new Wz4Mesh;
    m->Vertices.Add(vertices);
    vertices.Clear();

    // create a collider from this new mesh (convex hull shape)
    WpxCollider * col = new WpxCollider();
    col->Para.GeometryType = EGT_HULL;
    col->Para.StaticFriction = Para.StaticFriction;
    col->Para.DynamicFriction = Para.DynamicFriction;
    col->Para.Restitution = Para.Restitution;
    col->Para.Density = Para.Density;
    col->Para.Rot = sVector30(0.0f);
    col->Para.Trans = sVector31(0.0f);
    col->CreateGeometry(m);

    // create new chunkdebris object and fill Mesh and wCollider data
    sChunkCollider * d = new sChunkCollider;
    d->Mesh = m;
    d->wCollider = col;

    // add chunkdebris object to list
    ChunksColliders.AddTail(d);
  }

  return 0;
}

void WpxRigidBodyDebris::PhysxBuildDebris(const sMatrix34 & mat, PxScene * ptr)
{
  // for all chunkdebris object, create an actor

  sChunkCollider * d;
  sFORALL(ChunksColliders, d)
  {
    WpxRigidBody * rb = new WpxRigidBody();
    rb->AddRootCollider(d->wCollider);
    rb->Para.ActorType = Para.ActorType;
    rb->Para.MassAndInertia = Para.MassAndInertia;
    rb->Para.CenterOfMass = Para.CenterOfMass;
    rb->Para.Mass = Para.Mass;
    rb->Para.LinearVelocity = Para.LinearVelocity;
    rb->Para.AngularVelocity = Para.AngularVelocity;
    rb->Para.MaxAngularVelocity = Para.MaxAngularVelocity;
    rb->Para.SleepThreshold = Para.SleepThreshold;
    rb->Para.Sleep = Para.Sleep;
    rb->Para.Gravity = Para.Gravity;
    rb->Para.LinearDamping = Para.LinearDamping;
    rb->Para.AngularDamping = Para.AngularDamping;
    rb->Para.ForceMode = Para.ForceMode;
    rb->Para.Force = Para.Force;
    rb->Para.TorqueMode = Para.TorqueMode;
    rb->Para.Torque = Para.Torque;
    rb->Para.TimeFlag = Para.TimeFlag;
    rb->PhysxBuildActor(mat, ptr, AllActors);

    sVERIFY(RootNode!=0)

    // copy AllActor adress in RootNode
    WpxRigidBodyNodeDebris * rigidNode = static_cast<WpxRigidBodyNodeDebris *>(RootNode);
    if (rigidNode)
      rigidNode->AllActorsPtr = &AllActors;

    // AllActors is used in RenderNode to get global pose of physx actors
    // its pointer in RootNode must not be null
    sVERIFY(rigidNode!=0);
    sVERIFY(rigidNode->AllActorsPtr!=0);

    // store created rigidbody for clean delete
    ChunksRigidBodies.AddTail(rb);
  }
}

void WpxRigidBodyDebris::Transform(const sMatrix34 & mat, PxScene * ptr)
{
  sSRT srt;
  sMatrix34 mul, mulmat;

  srt.Scale = sVector31(1.0f);
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);

  mulmat = mul*mat;

  if (!ptr)
  {
    // ptr is null : transform is calling for WpxRigidBody Rendering

    // transform each chunk object to set the matrix used for rendering
    sChunkCollider * o;
    sFORALL(ChunksColliders, o)
      o->wCollider->Matrices.AddTail(sMatrix34CM(mulmat));
  }
  else
  {
    // ptr not null : transform is calling from Physx init to build physx objects
    PhysxBuildDebris(mulmat, ptr);
  }
}

/****************************************************************************/
/****************************************************************************/

template <typename T>
sINLINE void PhysxWakeUpRigidDynamics(sArray<sActor *> * actors, T &para)
{
  // called once at init
  // set initial forces and wake up actors

  PxRigidDynamic * rigidDynamic = 0;
  sActor * a;
  sFORALL(*actors, a)
  {
    if (para.ActorType == EAT_DYNAMIC)
    {
      rigidDynamic = static_cast<PxRigidDynamic*>(a->actor);

      // initial linear velocity
      PxVec3 linearVelocity(para.LinearVelocity.x, para.LinearVelocity.y, para.LinearVelocity.z);
      rigidDynamic->setLinearVelocity(linearVelocity);

      // initial angular velocity
      PxReal maxAngVel = para.MaxAngularVelocity;
      rigidDynamic->setMaxAngularVelocity(maxAngVel);
      PxVec3 angVel(para.AngularVelocity.x, para.AngularVelocity.y, para.AngularVelocity.z);
      rigidDynamic->setAngularVelocity(angVel);

      // force
      PxVec3 force(para.Force.x, para.Force.y, para.Force.z);
      PxForceMode::Enum forceMode = (PxForceMode::Enum)para.ForceMode;
      rigidDynamic->addForce(force, forceMode);

      // torque
      PxVec3 torque(para.Torque.x, para.Torque.y, para.Torque.z);
      PxForceMode::Enum torqueMode = (PxForceMode::Enum)para.TorqueMode;
      rigidDynamic->addTorque(torque, torqueMode);

      // wake up
      if (!para.Sleep)
        rigidDynamic->wakeUp();
      else
        rigidDynamic->putToSleep();
    }
  }
}

template <typename T>
sINLINE void SimulateRigidDynamics(sArray<sActor *> * actors, T &para)
{
  // called at each loop by simulate()
  // set rigidynamics properties and forces

  PxRigidDynamic *rigidDynamic = 0;
  sActor * a;

  sFORALL(*actors, a)
  {
    rigidDynamic = static_cast<PxRigidDynamic*>(a->actor);

    if (para.Sleep)
    {
      rigidDynamic->putToSleep();
    }
    else
    {
      // gravity flag
      bool gravityFlag = !para.Gravity;
      rigidDynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, gravityFlag);

      // damping
      PxReal linDamp = para.LinearDamping;
      rigidDynamic->setLinearDamping(linDamp);
      PxReal angDamp = para.AngularDamping;
      rigidDynamic->setAngularDamping(angDamp);

      // because theses forces are cumulated on each call of this function
      // we need to compensate call count based on real time (see: Physx::simulate())
      for (sInt j=0; j<gCumulatedCount; j++)
      {
        // force
        PxVec3 force(para.Force.x, para.Force.y, para.Force.z);
        PxForceMode::Enum forceMode = (PxForceMode::Enum)para.ForceMode;
        rigidDynamic->addForce(force, forceMode);

        // torque
        PxVec3 torque(para.Torque.x, para.Torque.y, para.Torque.z);
        PxForceMode::Enum torqueMode = (PxForceMode::Enum)para.TorqueMode;
        rigidDynamic->addTorque(torque, torqueMode);
      }
    }
  }
}

/****************************************************************************/
/****************************************************************************/

WpxRigidBodyNodeActor::WpxRigidBodyNodeActor()
{
  AllActorsPtr = 0;
}

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

WpxRigidBodyNodeDynamic::WpxRigidBodyNodeDynamic()
{
  Anim.Init(Wz4RenderType->Script);
}

void WpxRigidBodyNodeDynamic::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);

  if (Para.TimeFlag == 1)
    SimulateRigidDynamics(AllActorsPtr, Para);

  SimulateChilds(ctx);
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

WpxRigidBodyNodeDebris::WpxRigidBodyNodeDebris()
{
  ChunkedMeshPtr = 0;
  AllActorsPtr = 0;

  Anim.Init(Wz4RenderType->Script);
}

WpxRigidBodyNodeDebris::~WpxRigidBodyNodeDebris()
{
}

void WpxRigidBodyNodeDebris::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);

  if (Para.TimeFlag == 1)
    SimulateRigidDynamics(AllActorsPtr, Para);
}


void WpxRigidBodyNodeDebris::Transform(Wz4RenderContext *ctx, const sMatrix34 & mat)
{
  if (ctx)
  {
    // transformed by Wz4RenderNode process at each loop
    // get global pose of each debris and store their matrix

    sActor * a;
    PxTransform pT;
    sMatrix34 mmat;
    sMatrix34 matRes;
    sFORALL(*AllActorsPtr, a)
    {
      pT = a->actor->getGlobalPose();
      PxMat44TosMatrix34(pT, mmat);
      matRes = mmat;//*mat;
      *a->matrix = matRes;
    }
  }
  
  TransformChilds(ctx, mat);
}

void WpxRigidBodyNodeDebris::Render(Wz4RenderContext *ctx)
{
  // get nb chunks in mesh
  sInt max = ChunkedMeshPtr->Chunks.GetCount();

  // get nb of WpxRigidBodyNodeDebris instances (multiplied WpxRigidBodyNodeDebris)
  sInt nbMultiplied = AllActorsPtr->GetCount() / max;
  sInt indexInstance = 0;

#ifdef _DEBUG
  // nb actors should be a multiple of nb chunks
  // nb AllActor is nb chunck * nb of multiplied transformation (WpxRigidBodyMul multiply operator)
  sVERIFY(AllActorsPtr->GetCount() % max == 0);
#endif

  sMatrix34CM *mats0 = new sMatrix34CM[max];
  sMatrix34CM *matp;
  sMatrix34CM actorMat;

  // for all RenderNode transformations (yellow operators under physx)
  sFORALL(Matrices, matp)
  {
    // for every WpxRigidBodyNodeDebris instances
    for (sInt j=0; j<nbMultiplied; j++)
    {
      sActor ** a = AllActorsPtr->GetData();

      // for every actors per WpxRigidBodyNodeDebris instances
      for (sInt i=0; i<max; i++)
      {
        // compute result matrix
        actorMat = *a++[indexInstance]->matrix;
        mats0[i] = actorMat * (*matp);
      }

      // render once the chunked mesh with its matrices
      ChunkedMeshPtr->RenderBone(ctx->RenderMode, Para.EnvNum, max, mats0, max);

      // go to next WpxRigidBodyNodeDebris instance
      indexInstance += max;
    }

    // reset index instance per Matrices tranformations
    indexInstance = 0;
  }

  delete[] mats0;
}

/****************************************************************************/
/****************************************************************************/

// simple cache to save/restore list of physx particle nodes binded to a physx op
sArray<Wz4ParticleNode *> gPhysxParticleOperatorsCache;

RNPhysx::RNPhysx()
{
  Scene = 0;
  SceneTarget = 0;
  Accumulator = 0.0f;
  LastTime = 0;

  PreviousTimeLine = 0.0f;
  Executed = sFALSE;

  Anim.Init(Wz4RenderType->Script);
}

RNPhysx::~RNPhysx()
{
  // before destroy, cache all registered physx parts nodes
  gPhysxParticleOperatorsCache.Reset();
  gPhysxParticleOperatorsCache.Add(PartSystems);

  // delete physx scene
  if (Scene)
    Scene->release();

  // release target
  if (SceneTarget)
    SceneTarget->Release();

  // clean memory of all undeleted wpx operators
  WpxActorBase *c;
  sFORALL(WpxChilds,c)
  {
    c->Release();
  }
}

void RNPhysx::InitSceneTarget(PhysxTarget * target)
{
  // add and init target ptr
  SceneTarget = target;
  SceneTarget->AddRef();
  SceneTarget->PhysxSceneRef = Scene;
  SceneTarget->PartSystemsRef = &PartSystems;
}

sBool RNPhysx::Init(wCommand *cmd)
{
  // create a new physx scene
  Scene = CreateScene();
  if (!Scene)
    return sFALSE;

  // create all physx actors be recursing childs operators
  CreateAllActors(cmd);

  // pre-simulate and wake up physx scene
  WakeUpScene(cmd);

  return sTRUE;
}

void RNPhysx::CreateAllActors(wCommand *cmd)
{
  // create a first matrix
  sMatrix34 mat;
  mat.Init();

  // for each childs operators
  for (sInt i = 1; i<cmd->InputCount; i++)
  {
    WpxActorBase *in = cmd->GetInput<WpxActorBase *>(i);
    if (in)
    {
      // delete physx objects if it already exists
      if(!Doc->IsPlayer)
        in->PhysxReset();

      // process graph transformation, create physx objects and add them to physx scene
      in->Transform(mat, Scene);
      in->AddRef();

      // add wpx operator reference for delete process
      WpxChilds.AddTail(in);
    }
  }

  // restore cache if existing, to get list of all lost phsyx particles operators
  // it give the ability to reinit simulation of these operators when restarting animation (f6)
  PartSystems.Add(gPhysxParticleOperatorsCache);
}

void RNPhysx::WakeUpScene(wCommand *cmd)
{
  sF32 timeStep = 1.0f / 30.0f;

  // presimulation before wakeup (time to init stuff like joints...)
  if (Para.PreDelay)
  {
    for (sInt i = 0; i<Para.PreDelayCycles; i++)
    {
      Scene->simulate(timeStep);
      Scene->fetchResults(true);
    }
    Prepare(0);
  }

  // wake up actors
  for (sInt i = 1; i<cmd->InputCount; i++)
  {
    WpxActorBase *in = cmd->GetInput<WpxActorBase *>(i);
    if (in)
    {
      in->PhysxWakeUp();
    }
  }

  // presimulation after wakeup (advance all scene)
  if (Para.PreSimulation)
  {
    for (sInt i = 0; i<Para.SimulationCycles; i++)
    {
      Scene->simulate(timeStep);
      Scene->fetchResults(true);
      gCumulatedCount++;
    }
    Prepare(0);
  }
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

  if(Para.Desc&0x01)
    sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
  if(Para.Desc&0x04)
    sceneDesc.flags |= PxSceneFlag::eENABLE_ONE_DIRECTIONAL_FRICTION;
  if(Para.Desc&0x10)
    sceneDesc.flags |= PxSceneFlag::eENABLE_TWO_DIRECTIONAL_FRICTION;

  if (Para.ParticlesSimMode == 1)
  {
#ifdef PX_WINDOWS
    PxProfileZoneManager* mProfileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
    if (!mProfileZoneManager)
      sLogF(L"PhysX", L"PxProfileZoneManager::createProfileZoneManager failed!\n");

    pxtask::CudaContextManagerDesc cudaContextManagerDesc;
    pxtask::CudaContextManager * cudaContextManager = pxtask::createCudaContextManager(*gFoundation, cudaContextManagerDesc, mProfileZoneManager);

    if (cudaContextManager)
    {
      if (!cudaContextManager->contextIsValid())
      {
        cudaContextManager->release();
        cudaContextManager = NULL;
        sLogF(L"PhysX", L"Invalid CUDA context\n");
      }
      else
      {
        if (!sceneDesc.gpuDispatcher)
          sceneDesc.gpuDispatcher = cudaContextManager->getGpuDispatcher();
      }
    }
#endif
  }

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
  SimulateChilds(ctx);

  // reset gCumulatedCount forces counter
  gCumulatedCount = 0;

  // if not enabled, return;
  if (!Para.Enable)
    return;

  if (!Doc->IsPlayer)
  {
    // timeline paused ?
    if (App->TimelineWin->Pause)
      return;

    // compute elpased timeline to trigger the restart mechanism
    sF32 timeLine = ctx->GetBaseTime();
    sF32 deltaTimeLine = timeLine - PreviousTimeLine;
    PreviousTimeLine = timeLine;

    // Restart mechanism (for F6, loop demo, or clip restart)
    if (Executed && deltaTimeLine < -0.1f)
    {
      // rebuild operator (to restore default values)
      Doc->Change(Op, 1, 1);
      Executed = sFALSE;

      // rebuild all physx particles operators
      // reset list, because all rebuilded operator will go to register again
      sArray<Wz4ParticleNode *> listPartSystems(PartSystems);
      PartSystems.Reset();
      Wz4ParticleNode * p;
      sFORALL(listPartSystems, p)
        Doc->Change(p->Op, 1, 1);

      return;
    }
    Executed = sTRUE;
  }

  // simulation loop

  sF32 stepSize = 1.0f / sMax(10, Para.TimeStep);

  if (!Para.TimeSync)
  {
    Scene->simulate(stepSize);
    Scene->fetchResults(Para.WaitFetchResults);
    gCumulatedCount++;

  }
  else
  {
    // compute real elapsed time to synchronize simulation with real time
    static sTiming time;
    time.OnFrame(sGetTime());
    sF32 deltaTime = time.GetDelta() * Para.DeltaScale;
    // or
    /*sF32 newTime = sGetTimeUS() * 0.001;
    sF32 deltaTime = (newTime - LastTime) * Para.TimeScale;
    LastTime = newTime;*/

    if (Para.SyncMethod==0)
    {
      // simulation loop method 2
      Accumulator += deltaTime;

      if (Accumulator < stepSize)
        return;

      Scene->simulate(stepSize);
      Scene->fetchResults(Para.WaitFetchResults);

      gCumulatedCount++;
      Accumulator -= stepSize;
    }
    else
    {
      // simulation loop method 1
      if (deltaTime > Para.DeltaLimit)
        deltaTime = Para.DeltaLimit; // avoid spiral of death

      Accumulator += deltaTime;
      while (Accumulator >= stepSize)
      {
        Scene->simulate(stepSize);
        Scene->fetchResults(Para.WaitFetchResults);

        gCumulatedCount++;
        Accumulator -= stepSize;
      }
    }
  }

 //ViewPrintF(L"Scene actors : dynamic %d, static %d : %d\n",
 //    Scene->getNbActors(PxActorTypeSelectionFlag::eRIGID_DYNAMIC),
 //    Scene->getNbActors(PxActorTypeSelectionFlag::eRIGID_STATIC));
}

/****************************************************************************/
/****************************************************************************/
//
// PHYSX PARTICLES NODES
//
/****************************************************************************/
/****************************************************************************/

RPPhysxParticleTest::RPPhysxParticleTest()
{
  pIndex = 0;
  pPosition = 0;
  pVelocity = 0;
  PhysxPartSystem = 0;

  Anim.Init(Wz4RenderType->Script);
}

RPPhysxParticleTest::~RPPhysxParticleTest()
{
  // release physx particles and particle system
  PhysxPartSystem->releaseParticles();
  PhysxPartSystem->release();

  // delete buffers
  delete[] pIndex;
  delete[] pPosition;
  delete[] pVelocity;

  // remove this node from registered nodes for physx operator
  Target->RemoveParticleNode(this);
}

void RPPhysxParticleTest::Init()
{
  Para = ParaBase;

  // init particles pos
  Particles.AddMany(Para.Count);
  Particle *p;
  sRandomMT rnd;
  sFORALL(Particles, p)
  {
    p->Pos0.InitRandom(rnd);
    p->Pos1.InitRandom(rnd);
  }

  sVERIFY(Target);
  sVERIFY(Target->PhysxSceneRef);

  // particles buffers
  pIndex = new PxU32[Para.Count];
  pPosition = new PxVec3[Para.Count];
  pVelocity = new PxVec3[Para.Count];

  // fill buffers
  sFORALL(Particles, p)
  {
    p->Pos0 += Para.CloudPos;
    PxVec3 pos;
    pos.x = p->Pos0.x;
    pos.y = p->Pos0.y;
    pos.z = p->Pos0.z;

    p->Pos1 += Para.CloudPos;
    PxVec3 vel;
    pos.x = p->Pos1.x;
    pos.y = p->Pos1.y;
    pos.z = p->Pos1.z;

    pPosition[_i] = pos;
    pVelocity[_i] = vel;
    pIndex[_i] = _i; 
  }

  // declare particle descriptor for creating new particles
  PxParticleCreationData particleCreationData;
  particleCreationData.numParticles = Para.Count;
  particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(pIndex);
  particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(pPosition);
  particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(pVelocity);

  // create particle system
  PhysxPartSystem = gPhysicsSDK->createParticleSystem(Para.Count);
  if(PhysxPartSystem)
  {
    PhysxPartSystem->setParticleBaseFlag(PxParticleBaseFlag::eGPU, true);

    // add particle system to physx scene
    Target->PhysxSceneRef->addActor(*PhysxPartSystem);

    // create particles in particle system 
    bool success = PhysxPartSystem->createParticles(particleCreationData);
  }
}

void RPPhysxParticleTest::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);
}

sInt RPPhysxParticleTest::GetPartCount()
{
  return Particles.GetCount();
}
sInt RPPhysxParticleTest::GetPartFlags()
{
  return 0;
}

void RPPhysxParticleTest::Func(Wz4PartInfo &pinfo, sF32 time, sF32 dt)
{
  sVector31 p;
  Particle *part; 

  // lock SDK buffers of *PxParticleSystem* for reading
  PxParticleReadData* rd = PhysxPartSystem->lockParticleReadData();

  // access particle data from PxParticleReadData
  PxStrideIterator<const PxParticleFlags> flagsIt(rd->flagsBuffer);
  PxStrideIterator<const PxVec3> positionIt(rd->positionBuffer);

  sFORALL(Particles, part)
  {
    if (*flagsIt & PxParticleFlag::eVALID)
    {
      const PxVec3& position = *positionIt;

      p.x = position.x;
      p.y = position.y;
      p.z = position.z;
      //p += sVector30(part->Pos1) + Para.CloudPos;

      pinfo.Parts[_i].Init(p, time);

      // increment iterators
      positionIt++;
      flagsIt++;
    }
  }

  rd->unlock();

  pinfo.Used = pinfo.Alloc;
}

/****************************************************************************/




RPPxPart::RPPxPart()
{
  pIndex = 0;
  pPosition = 0;
  pVelocity = 0;
  PhysxPartSystem = 0;
  Source = 0;

  Anim.Init(Wz4RenderType->Script);


  NeedInit = sTRUE;
  
}

RPPxPart::~RPPxPart()
{
  // release physx particles and particle system
  PhysxPartSystem->releaseParticles();
  PhysxPartSystem->release();

  // delete buffers
  delete[] pIndex;
  delete[] pPosition;
  delete[] pVelocity;

  // remove this node from registered nodes for physx operator
  Target->RemoveParticleNode(this);

  Source->Release();
}

void RPPxPart::Init()
{
  Para = ParaBase;

  if (Source)
    Particles.AddMany(Source->GetPartCount());

  return;


  // init particles pos
  //Particles.AddMany(Para.Count);
  Particle *p;
  sRandomMT rnd;
  sFORALL(Particles, p)
  {
    p->Pos0.InitRandom(rnd);
    p->Pos1.InitRandom(rnd);
  }

  sVERIFY(Target);
  sVERIFY(Target->PhysxSceneRef);

  // particles buffers
  pIndex = new PxU32[Para.Count];
  pPosition = new PxVec3[Para.Count];
  pVelocity = new PxVec3[Para.Count];

  // fill buffers
  sFORALL(Particles, p)
  {    
    p->Pos0 += Para.CloudPos;
    PxVec3 pos;
    pos.x = p->Pos0.x;
    pos.y = p->Pos0.y;
    pos.z = p->Pos0.z;

    //p->Pos1 += Para.CloudFreq;
    PxVec3 vel;
    vel.x = p->Pos1.x + Para.CloudFreq[0];
    vel.y = p->Pos1.y + Para.CloudFreq[1];
    vel.z = p->Pos1.z + 0;

    pPosition[_i] = pos;
    pVelocity[_i] = pos;
    pIndex[_i] = _i;
  }

  // declare particle descriptor for creating new particles
  PxParticleCreationData particleCreationData;
  particleCreationData.numParticles = Para.Count;
  particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(pIndex);
  particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(pPosition);
  particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(pVelocity);

  // create particle system
  PhysxPartSystem = gPhysicsSDK->createParticleSystem(Para.Count);
  if (PhysxPartSystem)
  {
    PhysxPartSystem->setParticleBaseFlag(PxParticleBaseFlag::eGPU, true);
    PhysxPartSystem->setGridSize(3.0f);
    PhysxPartSystem->setMaxMotionDistance(0.43f);
    PhysxPartSystem->setRestOffset(0.0143f);
    PhysxPartSystem->setContactOffset(0.0143f * 2);
    PhysxPartSystem->setDamping(0.0f);
    PhysxPartSystem->setRestitution(0.2f);
    PhysxPartSystem->setDynamicFriction(0.05f);
    PhysxPartSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, true);


    

    // add particle system to physx scene
    Target->PhysxSceneRef->addActor(*PhysxPartSystem);

    // create particles in particle system 
    bool success = PhysxPartSystem->createParticles(particleCreationData);
  }
}

void RPPxPart::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);

 // if (Source)
  //  Source->Simulate(ctx);

  if (NeedInit)
    DelayedInit();
}

sINLINE void sVector31ToPxVec3(PxVec3& v, sVector31Arg& w)
{ 
  v.x = w.x;
  v.y = w.y;
  v.z = w.z;
}

sINLINE void sVector30ToPxVec3(PxVec3& v, sVector30Arg& w)
{
  v.x = w.x;
  v.y = w.y;
  v.z = w.z;
}

sINLINE void PxVec3TosVector31(sVector31& w, PxVec3& v)
{
  w.x = v.x;
  w.y = v.y;
  w.z = v.z;
}




void RPPxPart::DelayedInit()
{
  sInt maxsrc = Source->GetPartCount();

  // run source one time
  Wz4PartInfo part;
  part.Init(Source->GetPartFlags(), maxsrc);
  part.Reset();
  Source->Func(part, 0, 0.0f);

  // create particles buffers
  bStartPosition = new PxVec3[maxsrc]; 
  pIndex = new PxU32[maxsrc];
  pPosition = new PxVec3[maxsrc];
  pVelocity = new PxVec3[maxsrc];

  // fill buffers
  Particle *p;
  sFORALL(Particles, p)
  {
    sVector31ToPxVec3(bStartPosition[_i], part.Parts[_i].Pos);
    sVector31ToPxVec3(pPosition[_i], part.Parts[_i].Pos);    
    sVector31ToPxVec3(pVelocity[_i], part.Parts[_i].Pos);
    pIndex[_i] = _i;
  }







  // declare particle descriptor for creating new particles
  PxParticleCreationData particleCreationData;
  particleCreationData.numParticles = Para.Count;
  particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(pIndex);
  particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(pPosition);
  particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(pVelocity);

  // create particle system
  PhysxPartSystem = gPhysicsSDK->createParticleSystem(Para.Count);
  if (PhysxPartSystem)
  {
    PhysxPartSystem->setParticleBaseFlag(PxParticleBaseFlag::eGPU, true);
    PhysxPartSystem->setGridSize(3.0f);
    PhysxPartSystem->setMaxMotionDistance(0.43f);
    PhysxPartSystem->setRestOffset(0.0143f);
    PhysxPartSystem->setContactOffset(0.0143f * 2);
    PhysxPartSystem->setDamping(0.0f);
    PhysxPartSystem->setRestitution(0.2f);
    PhysxPartSystem->setDynamicFriction(0.05f);
    PhysxPartSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, true);

    // add particle system to physx scene
    Target->PhysxSceneRef->addActor(*PhysxPartSystem);

    // create particles in particle system 
    bool success = PhysxPartSystem->createParticles(particleCreationData);
  }


  NeedInit = sFALSE;





  /*sRandom rnd;
  rnd.Seed(Para.RandomSeed);
  sInt maxsrc = Source->GetPartCount();

  Wz4PartInfo part[2];
  sInt db = 0;

  part[0].Init(Source->GetPartFlags(), maxsrc);
  part[1].Init(Source->GetPartFlags(), maxsrc);

  part[db].Reset();
  Source->Func(part[db], -1.0f, 0);

  Sparcs.Clear();
  for (sInt i = 0; i<Para.SamplePoints; i++)
  {
    db = !db;
    sF32 time = sF32(i) / Para.SamplePoints;

    if (Para.Distribution & 1)
      time /= Para.Percentage;

    part[db].Reset();
    Source->Func(part[db], time + Para.Delay, 0);

    for (sInt j = 0; j<maxsrc; j++)
    {
      sBool create = rnd.Float(1)<Para.Percentage;
      if (Para.Distribution & 1)
        create = sTRUE;

      if (part[db].Parts[j].Time >= 0 && create && Sparcs.GetCount()<MaxSparks)
      {
        sVector30 speed;
        Sparc *s = Sparcs.AddMany(1);

        s->Time0 = time;
        s->Pos = part[db].Parts[j].Pos;
        speed.InitRandom(rnd);
        s->Speed = speed*Para.RandomSpeed;
        speed = part[db].Parts[j].Pos - part[!db].Parts[j].Pos;
        speed.Unit();
        s->Speed += speed*Para.DirectionSpeed;
      }
    }
  }

  NeedInit = sFALSE;*/
}

sInt RPPxPart::GetPartCount()
{
  return Particles.GetCount();
}
sInt RPPxPart::GetPartFlags()
{
  return Source ? Source->GetPartFlags() : 0;
}

void RPPxPart::Func(Wz4PartInfo &pinfo, sF32 time, sF32 dt)
{
  sVector31 p;
  Particle *part;


  if (Source)
    Source->Func(pinfo, time, dt);





  sInt maxSource = Source->GetPartCount();
  PxU32 * bIndices = new PxU32[maxSource];
  PxVec3 * bPositions = new PxVec3[maxSource];
  sInt nbCount = 0;
  sFORALL(Particles, part)
  {
    if (pinfo.Parts[_i].Time >= 0.95)
    {
      //indices.AddTail(_i);
      //positions.AddTail(bStartPosition[_i]);

      pinfo.Parts[_i].Time = -1;
      PxVec3TosVector31(pinfo.Parts[_i].Pos, bStartPosition[_i]);

      bIndices[nbCount] = nbCount;
      bPositions[nbCount] = bStartPosition[_i];
      nbCount++;
    }
  }

  PxStrideIterator<const PxU32> bIndicesIt(bIndices);
  PxStrideIterator<const PxVec3> bPositionsIt(bPositions);
  PhysxPartSystem->setPositions(nbCount, bIndicesIt, bPositionsIt);
















  // lock SDK buffers of *PxParticleSystem* for reading
  PxParticleReadData* rd = PhysxPartSystem->lockParticleReadData();

  // access particle data from PxParticleReadData
  PxStrideIterator<const PxParticleFlags> flagsIt(rd->flagsBuffer);
  PxStrideIterator<const PxVec3> positionIt(rd->positionBuffer);  
  PxStrideIterator<const PxVec3> velIt(rd->velocityBuffer);
  
  PxStrideIterator<const PxVec3> gg(bStartPosition);
  
  //PhysxPartSystem->addForces(Source->GetPartCount(), dd, gg, PxForceMode::eVELOCITY_CHANGE);



  
  

  sFORALL(Particles, part)
  {
    if (_i >= Source->GetPartCount() - 1)
      break;

    if (*flagsIt & PxParticleFlag::eVALID)
    {
      const PxVec3& position = *positionIt;

      p.x = position.x;
      p.y = position.y;
      p.z = position.z;
      //p += sVector30(part->Pos1) + Para.CloudPos;

      sF32 t = 1;
      if (Source)
        t = pinfo.Parts[_i].Time;
      
/*p.x += velIt->x; // sVector30(pinfo.Parts[_i].Pos);
      p.y += velIt->y;
      p.z += velIt->z;

      velIt++;*/

     
      /*if (t > 0.9)
      {
        PxVec3TosVector31(p, *bStartPosition);
      }
      bStartPosition++;*/

      


      pinfo.Parts[_i].Init(p, t);

      // increment iterators
      positionIt++;
      flagsIt++;
    }
  }

  rd->unlock();



  


  pinfo.Used = pinfo.Alloc;
}

/****************************************************************************/

int RandU(int nMin, int nMax)
{
  return nMin + (int)((double)rand() / (RAND_MAX + 1) * (nMax - nMin + 1));
}

sF32 RandF(sF32 nMin, sF32 nMax)
{
  return (nMax - nMin) * ((((float)rand()) / (float)RAND_MAX)) + nMin;
}

sVector30 Randv30(sVector30 nMin, sVector30 nMax)
{
  sVector30 v;
  v.x = (nMax.x - nMin.x) * ((((float)rand()) / (float)RAND_MAX)) + nMin.x;
  v.y = (nMax.y - nMin.y) * ((((float)rand()) / (float)RAND_MAX)) + nMin.y;
  v.z = (nMax.z - nMin.z) * ((((float)rand()) / (float)RAND_MAX)) + nMin.z;
  return v;
}

sVector31 Randv31(sVector31 nMin, sVector31 nMax)
{
  sVector31 v;
  v.x = (nMax.x - nMin.x) * ((((float)rand()) / (float)RAND_MAX)) + nMin.x;
  v.y = (nMax.y - nMin.y) * ((((float)rand()) / (float)RAND_MAX)) + nMin.y;
  v.z = (nMax.z - nMin.z) * ((((float)rand()) / (float)RAND_MAX)) + nMin.z;
  return v;
}

RPEmiter::RPEmiter()
{
  Anim.Init(Wz4RenderType->Script);
  AccumultedTime = 0;
}

RPEmiter::~RPEmiter()
{
}

void RPEmiter::Init()
{
  Para = ParaBase;
  Particles.AddMany(Para.MaxParticles);
}

void RPEmiter::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase;
  Anim.Bind(ctx->Script, &Para);
  SimulateCalc(ctx);

  if (ctx->GetTime() <= 0 || App->TimelineWin->Pause)
    return;

  // get delta time
  static sTiming timer;
  timer.OnFrame(sGetTime());
  sF32 deltaTime = timer.GetDelta() * 0.001;

  Particle * p;
  sFORALL(Particles, p)
  {
    if (p->Life >= sMin(p->MaxLife, 0.99f))
    {
      // Release old particles
      p->isDead = sTRUE;
      p->Life = -1;
      p->Position = sVector31(0);
    }
    else if (!p->isDead)
    {
      // update living particles
      p->Position = p->Position + (p->Velocity * deltaTime);
      p->Life = p->Life + deltaTime;
    }
  }

  // Emit new particles

  sInt rateValue = Para.Rate;
  if (Para.RateDistribution)
    rateValue = RandU(Para.RateRangeMin, Para.RateRangeMax);
  sF32 rate = 1.0f / rateValue;

  sBool emitParticle;
  AccumultedTime += deltaTime;

  if (AccumultedTime < rate)
    return;

  // if it's time to emit particle

  // search for a dead particle
  sFORALL(Particles, p)
  {
    // if dead one, rebirth it
    if (p->isDead)
    {
      p->isDead = sFALSE;
      p->Life = 0;

      // max life
      p->MaxLife = Para.MaxLife;
      if (Para.MaxLifeDistribution)
        p->MaxLife = RandF(Para.MaxLifeRangeMin, Para.MaxLifeRangeMax);

      // location
      p->Position = Para.Location;
      if (Para.LocationDistribution)
        p->Position = Randv31(Para.LocationRangeMin, Para.LocationRangeMax);

      // velocity
      p->Velocity = Para.Velocity;
      if (Para.VelocityDistribution)
        p->Velocity = Randv30(Para.VelocityRangeMin, Para.VelocityRangeMax);

      break;
    }
  }
  AccumultedTime -= rate;
}

sInt RPEmiter::GetPartCount()
{
  return Particles.GetCount();
}
sInt RPEmiter::GetPartFlags()
{
  return 0;
}

void RPEmiter::Func(Wz4PartInfo &pinfo, sF32 time, sF32 dt)
{
  Particle * p;
  sFORALL(Particles, p)
  {
    pinfo.Parts[_i].Init(p->Position, p->Life);
  }

  pinfo.Used = pinfo.Alloc;
}
