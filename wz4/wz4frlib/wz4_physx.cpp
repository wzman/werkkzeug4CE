/*+**************************************************************************/
/***                                                                      ***/
/***   This file is distributed under a BSD license.                      ***/
/***   See LICENSE.txt for details.                                       ***/
/***                                                                      ***/
/**************************************************************************+*/

#include "wz4_physx.hpp"

/****************************************************************************/
// GLOBAL VARIABLES AND FUNCTIONS
/****************************************************************************/

PxFoundation * gFoundation;   // global physx foundation pointer
PxPhysics * gPhysicsSDK;      // global physx engine pointer
PxCooking * gCooking;         // global physx cooking pointer
sInt gCumulatedCount = 0;     // cumulated forces counter for animated rigid bodies

static PxDefaultErrorCallback gDefaultErrorCallback;
static PxDefaultAllocator gDefaultAllocatorCallback;
static PxSimulationFilterShader gDefaultFilterShader=PxDefaultSimulationFilterShader;

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
}

/****************************************************************************/
// HELPERS
/****************************************************************************/

MemoryOutputStream::MemoryOutputStream() :
	mData		(NULL),
	mSize		(0),
	mCapacity	(0)
{
}

MemoryOutputStream::~MemoryOutputStream()
{
	if(mData)
		delete[] mData;
}

PxU32 MemoryOutputStream::write(const void* src, PxU32 size)
{
	PxU32 expectedSize = mSize + size;
	if(expectedSize > mCapacity)
	{
		mCapacity = expectedSize + 4096;

		PxU8* newData = new PxU8[mCapacity];
		PX_ASSERT(newData!=NULL);

		if(newData)
		{
			memcpy(newData, mData, mSize);
			delete[] mData;
		}
		mData = newData;
	}
	memcpy(mData+mSize, src, size);
	mSize += size;
	return size;
}

/****************************************************************************/

MemoryInputData::MemoryInputData(PxU8* data, PxU32 length) :
	mSize	(length),
	mData	(data),
	mPos	(0)
{
}

PxU32 MemoryInputData::read(void* dest, PxU32 count)
{
	PxU32 length = PxMin<PxU32>(count, mSize-mPos);
	memcpy(dest, mData+mPos, length);
	mPos += length;
	return length;
}

PxU32 MemoryInputData::getLength() const
{
	return mSize;
}

void MemoryInputData::seek(PxU32 offset)
{
	mPos = PxMin<PxU32>(mSize, offset);
}

PxU32 MemoryInputData::tell() const
{
	return mPos;
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

PXShapeCollider::PXShapeCollider() 
{ 
  ShapeMesh = 0; 
  ConvexMesh = 0; 
  TriMesh = 0; 
}

PXShapeCollider::~PXShapeCollider() 
{ 
  if(ShapeMesh) delete ShapeMesh;
  if(ConvexMesh) ConvexMesh->release();
  if(TriMesh) TriMesh->release();
}

void PXShapeCollider::Init(Wz4Mesh * inMesh)
{
  sVector31 ShapeSize(1.0f);

  ShapeMesh = new Wz4Mesh();
  ShapeMesh->AddRef();    

  switch(Para.GeometryType)
  {
  case EGT_CUBE:
    ShapeMesh->MakeCube(1,1,1);
    ShapeSize = Para.Dimension;
    break;

  case EGT_SPHERE:
    ShapeMesh->MakeSphere(12,12);
    ShapeSize = sVector31(Para.Radius);
    break;

  case EGT_PLANE:
    ShapeMesh->MakeGrid(16,16);
    break;

  case EGT_HULL:
    ConvexMesh = MakePxConvexHull(inMesh);
    if(ConvexMesh == 0)
    {
      // failed to create PxConvexMesh
      delete ShapeMesh;
      ShapeMesh = 0;
      return;
    }
    else
    {
      PxConvexMeshToWz4Mesh(ConvexMesh, ShapeMesh);
    }
    break;

  case EGT_MESH:
    TriMesh = MakePxMesh(inMesh);
    if(TriMesh == 0)
    {
      // failed to create PxConvexMesh
      delete ShapeMesh;
      ShapeMesh = 0;
      return;
    }
    else
    {
      PxTriMeshToWz4Mesh(TriMesh, ShapeMesh);
    }    
    break;
  }

  // center shape, except for hull or mesh
  if(Para.GeometryType != EGT_HULL && Para.GeometryType != EGT_MESH)
  {    
    Wz4MeshVertex *mv;
    sAABBox bounds;
    sFORALL(ShapeMesh->Vertices,mv)
      bounds.Add(mv->Pos);
    sVector30 d = (sVector30(bounds.Max)+sVector30(bounds.Min))*-0.5f;
    sFORALL(ShapeMesh->Vertices,mv)
      mv->Pos += d;
    ShapeMesh->Flush();
  }

  // apply transformation matrix to mesh
  sMatrix34 mul;
  sSRT srt;
  srt.Scale = ShapeSize;
  srt.Rotate = Para.Rot; 
  srt.Translate = Para.Trans;
  srt.MakeMatrix(mul);
  ShapeMesh->Transform(mul);

  // set material
  Wz4MeshCluster * cluster = new  Wz4MeshCluster();
  ShapeMesh->Clusters.AddTail(cluster);
  SimpleMtrl *mtrl = new SimpleMtrl;
  mtrl->SetMtrl(sMTRL_ZOFF|sMTRL_CULLOFF|sMTRL_MSK_GREEN|sMTRL_MSK_BLUE, 0 );
  mtrl->Prepare();
  ShapeMesh->Clusters[0]->Mtrl = mtrl;

  // fill and store collider structure info
  sColliderObj colData;
  colData.ShapePtr = ShapeMesh;
  colData.ParaPtr = &Para;
  colData.ConvexMeshPtr = ConvexMesh;
  colData.TriMeshPtr = TriMesh;
  ColliderData.AddTail(colData);
}

/****************************************************************************/

PXShapeAdd::PXShapeAdd()
{
} 

PXShapeAdd::~PXShapeAdd()
{
}

void PXShapeAdd::AddCollider(WpxCollider* colOp)
{
  PXShapeCollider * col = (PXShapeCollider*)colOp->RootNode;
  ColliderData.Add(col->ColliderData);
}

/****************************************************************************/

PXRigidMesh::PXRigidMesh()
{
  Joints = 0;
  JointMesh = 0;
  PtrKinematicActorList = 0;
  PtrAnimatedActorList = 0;
  Anim.Init(Wz4RenderType->Script);
}

PXRigidMesh::~PXRigidMesh()
{
  if(PtrKinematicActorList) PtrKinematicActorList->Reset();
  if(PtrAnimatedActorList) PtrAnimatedActorList->Reset();

  // delete all created mesh in vertices build mode
  for(sInt i=0; i<CreatedMeshArray.GetCount(); i++)
    delete CreatedMeshArray[i];

  // delete all created matrix in vertices build mode
  for(sInt i=0; i<CreatedMatArray.GetCount(); i++)
    delete CreatedMatArray[i];

  // delete all created matrix in vertices build mode
  for(sInt i=0; i<JointsMatrix.GetCount(); i++)
    delete JointsMatrix[i];

  // delete joint mesh model
  delete JointMesh;
}

void PXRigidMesh::Simulate(Wz4RenderContext *ctx)
{
  Para = ParaBase; 
  Anim.Bind(ctx->Script,&Para);
  SimulateCalc(ctx);

  // animate kinematics actors
  if(PtrKinematicActorList)
  {
    for(sInt i=0; i<PtrKinematicActorList->GetCount(); i++)
    {     
      sRigidDynamicActor * actors = PtrKinematicActorList->GetData();    

      sMatrix34 wzMat34, wz2;
      sSRT srt;
      srt.Scale = sVector31(1.0f);
      srt.Rotate = actors[i].ActorPtr->ParaPtr->Rot;
      srt.Translate = actors[i].ActorPtr->ParaPtr->Trans;
      srt.MakeMatrix(wzMat34);

      //wzMat34 *= *actors[i].ActorPtr->GlobalMatrixPtr; 

      PxMat44 pxMat;
      sMatrix34ToPxMat44(wzMat34, pxMat);
      PxTransform transform(pxMat);

      actors[i].RigidDynamicPtr->setKinematicTarget(transform);
    }
  }
  
  // dynamics actors
  if(PtrAnimatedActorList)
  {
    // animate rigid dynamic actors
    for(sInt i=0; i<PtrAnimatedActorList->GetCount(); i++)
    {
      sRigidDynamicActor * actors = PtrAnimatedActorList->GetData();
      WpxActorParaRigidBody  * para = actors[i].ActorPtr->ParaPtr;
      PxRigidDynamic * rigidDynamic = actors[i].RigidDynamicPtr;
      
      if(para->Sleep)
      { 
        // go to bed !
        if(!rigidDynamic->isSleeping())
          rigidDynamic->putToSleep();
      }
      else
      {
        if(para->TimeFlag == 1)
        {
          // gravity flag
          bool gravityFlag = !para->Gravity;
          rigidDynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, gravityFlag);

          // damping 
          PxReal linDamp = para->LinearDamping;
          rigidDynamic->setLinearDamping(linDamp);
          PxReal angDamp = para->AngularDamping;
          rigidDynamic->setAngularDamping(angDamp);

          // because theses forces are cumulated on each call of this function
          // we need to compensate call count based on real time (see: Physx::simulate())
          for(sInt j=0; j<gCumulatedCount; j++)
          {      
            // force
            PxVec3 force(para->Force.x, para->Force.y, para->Force.z);
            PxForceMode::Enum forceMode = (PxForceMode::Enum)para->ForceMode;
            rigidDynamic->addForce(force, forceMode);

            // torque
            PxVec3 torque(para->Torque.x, para->Torque.y, para->Torque.z);
            PxForceMode::Enum torqueMode = (PxForceMode::Enum)para->TorqueMode;
            rigidDynamic->addTorque(torque, torqueMode);
          }
        }
      }
    }
  }
}

void PXRigidMesh::Transform(Wz4RenderContext *ctx,const sMatrix34 &mat)
{
}

void PXRigidMesh::BuildRigidBodyFromMesh(WpxCollider * in, Wz4Mesh * mesh)
{  
  PXShapeCollider * col = (PXShapeCollider*)in->RootNode;

  sSRT srt;
  srt.Scale = sVector31(1,1,1); //para->Scale;
  srt.Rotate = Para.Rot;
  srt.Translate = Para.Trans;
  srt.MakeMatrix(Matrix);

  // fill and store collider structure info
  sActorObj actorData;
  actorData.MeshPtr = mesh;
  actorData.ParaPtr = &Para;
  actorData.GlobalMatrixPtr = &Matrix;
  actorData.ColliderDataPtr = &col->ColliderData;
  actorData.ActorId = Para.ActorID;
  ActorData.AddTail(actorData);

  // get pointer to kinematics actors list
  PtrKinematicActorList = &KinematicActorList;

  // get pointer to dynamics actors list
  PtrAnimatedActorList = &AnimatedActorPtrList; 

}

void PXRigidMesh::BuildRigidBodyFromVertices(WpxCollider * in, Wz4Mesh * mesh, Wz4Mesh * meshModel)
{
  PXShapeCollider * col = (PXShapeCollider*)in->RootNode;
  
  sArray<sVector31> ListVertices;
  Wz4MeshVertex * v;
  sVector31 * vect;

  sRandom rnd(Para.Seed); 

  // get unique vertices of input mesh    
  sFORALL(mesh->Vertices, v)
  {
    if(!sContains(ListVertices, v->Pos))
      ListVertices.AddTail(v->Pos);      
  }

  // compute actor global pose matrix
  sSRT s;
  s.Scale = sVector31(1.0f);
  s.Rotate = Para.Rot;
  s.Translate = Para.Trans;
  s.MakeMatrix(Matrix);

  sBool isFirstMesh = sTRUE;
  sInt firstActorIndex = -1;
  sInt nbMeshCreated = 0;

  // place a new mesh based on the meshmodel at each vertex pose    
  sFORALL(ListVertices, vect)
  {
    if(rnd.Float(1) <= Para.Random)
    {  
      // keep index of the first actor to be inserted in the array
      // to update it later with the correct nbr of mesh instance
      if(isFirstMesh)
      {
        firstActorIndex = ActorData.GetCount();
        isFirstMesh = sFALSE;
      }
      nbMeshCreated++;
      
      // compute the new mesh matrix
      sMatrix34  * mat = new sMatrix34;  
      sSRT srt;      
      srt.Scale = sVector31(1.0f);
      srt.Rotate = Para.RotVertices;
      srt.Translate = *vect;
      srt.MakeMatrix(*mat);
      CreatedMatArray.AddTail(mat);     

      // multiply mesh matrix with actor pose
      *mat = *mat* Matrix;

      // create new mesh from model
      Wz4Mesh  * newMesh = new Wz4Mesh;
      newMesh->CopyFrom(meshModel);
      CreatedMeshArray.AddTail(newMesh);

      // fill and store the collider structure info
      sActorObj actorData;
      actorData.MeshPtr = newMesh;
      actorData.ParaPtr = &Para;
      actorData.GlobalMatrixPtr = mat;
      actorData.ColliderDataPtr = &col->ColliderData;
      actorData.InstanceCount = 0;
      actorData.ActorId = Para.ActorID + _i;
      ActorData.AddTail(actorData);

      // get pointer to kinematics actors list
      PtrKinematicActorList = &KinematicActorList;

      // get pointer to dynamics actors list
      PtrAnimatedActorList = &AnimatedActorPtrList;
    }
  }

  // update correct nbr of instances created of this actor
  if(firstActorIndex != -1)
    ActorData[firstActorIndex].InstanceCount = nbMeshCreated;
}


void PXRigidMesh::CreateJointMeshViewer()
{  
  sSRT srt;
  sMatrix34 mat;

  // create a cube
  JointMesh = new Wz4Mesh;
  JointMesh->MakeCube(1,1,1);   

  // create a smaller cube
  Wz4Mesh * m = new Wz4Mesh;
  m->MakeCube(1,1,1);
  srt.Scale = sVector31(0.5, 0.5, 0.5);
  srt.Translate = sVector31(-0.5,0.25,0.25);
  srt.MakeMatrix(mat);
  m->Transform(mat);

  // merge both cubes
  JointMesh->Add(m);
  m->Release();

  // scale result mesh
  srt.Scale = sVector31(0.1);
  srt.Translate = sVector31(0,0,0);
  srt.MakeMatrix(mat);
  JointMesh->Transform(mat);

  // center overall
  Wz4MeshVertex *mv;
  sAABBox bounds;
  sFORALL(JointMesh->Vertices,mv)
    bounds.Add(mv->Pos);
  sVector30 d = (sVector30(bounds.Max)+sVector30(bounds.Min))*-0.5f;
  sFORALL(JointMesh->Vertices,mv)
    mv->Pos += d;
  JointMesh->Flush();
}


void PXRigidMesh::BuildJoints(WpxActorArrayRigidBody * array, sInt arrayCount)
{ 
  // create a mesh to preview joints contact point
  CreateJointMeshViewer();
  
  // copy array ptr adress
  WpxActorArrayRigidBody * adress = array;

  // for each element in array build a matrix
  for(sInt j=0; j<ActorData.GetCount(); j++)
  {
    array = adress;

    for(sInt i=0; i<arrayCount; i++)
    {   
      sMatrix34  * wzMat = new sMatrix34;
      sSRT srt;
      srt.Scale = sVector31(1.0f);
      srt.Rotate = array->Rot;
      srt.Translate = array->Trans;
      srt.MakeMatrix(*wzMat);

      //wzMat = *ActorData[j].GlobalMatrixPtr;

      PxMat44 pxMat;
      sMatrix34ToPxMat44(*wzMat, pxMat);
      PxTransform t(pxMat);
      ActorData[j].JointsPoses.AddTail(t);

      sMatrix34 rigidBodyPose;
      sSRT srt2;
      srt2.Scale = sVector31(1.0f);
      srt2.Rotate = Para.Rot;
      srt2.Translate = Para.Trans;
      srt2.MakeMatrix(rigidBodyPose);

      //wzMat *= rigidBodyPose;
      //wzMat *= *ActorData[j].GlobalMatrixPtr;
      *wzMat *= *ActorData[j].GlobalMatrixPtr;

      JointsMatrix.AddTail(wzMat); 

      array++;
    }
  }

  for(sInt i=0; i<ActorData.GetCount(); i++)
  {
    ActorData[i].JointsMatrixArrayPtr = &JointsMatrix;
    ActorData[i].JointMeshPtr = JointMesh; //JointMesh->AddRef();
  }
}


/****************************************************************************/

PXActorAdd::PXActorAdd()
{
}

PXActorAdd::~PXActorAdd()
{
}

void PXActorAdd::Simulate(Wz4RenderContext *ctx)
{
  SimulateChilds(ctx);
}

void PXActorAdd::AddActor(WpxActor * actOp)
{
  PXRigidMesh * act = (PXRigidMesh*)actOp->RootNode;
  ActorData.Add(act->ActorData);

  // copy kinematic actor list ptr
  PtrKinematicActorList = act->PtrKinematicActorList;

  // copy animated actor list ptr
  PtrAnimatedActorList = act->PtrAnimatedActorList;
}

/****************************************************************************/

PXActorMultiply::PXActorMultiply()
{
} 

void PXActorMultiply::Simulate(Wz4RenderContext *ctx)
{
  SimulateChilds(ctx);
}

PXActorMultiply::~PXActorMultiply() 
{
  // delete all created matrix
  for(sInt i=0; i<MulMatArray.GetCount(); i++)
    delete MulMatArray[i];

  // delete all created sActorObj 
  for(sInt i=0; i<ActorDataArray.GetCount(); i++)
    delete ActorDataArray[i];
}  

/****************************************************************************/

RNPhysx::RNPhysx()
{
  Executed = sFALSE;
  ActorBuffer = 0;
  mScene = 0;
  Anim.Init(Wz4RenderType->Script);
}

RNPhysx::~RNPhysx()
{
  if(ActorBuffer) delete ActorBuffer;
  if(mScene) mScene->release();
}

sBool RNPhysx::CreateScene()
{
  // Create the scene
  PxSceneDesc sceneDesc(gPhysicsSDK->getTolerancesScale());
  sceneDesc.gravity = PxVec3(Para.Gravity.x, Para.Gravity.y, Para.Gravity.z);
  if(!sceneDesc.cpuDispatcher)
  {
    PxDefaultCpuDispatcher* mCpuDispatcher = PxDefaultCpuDispatcherCreate(Para.NumThreads);

    if(!mCpuDispatcher)
    {
      sLogF(L"PhysX",L"CreateScene - PxDefaultCpuDispatcherCreate failed!\n");
      return sFALSE;
    }

    sceneDesc.cpuDispatcher = mCpuDispatcher;
  } 

  if(!sceneDesc.filterShader)
    sceneDesc.filterShader  = gDefaultFilterShader;

  
  mScene = gPhysicsSDK->createScene(sceneDesc);

  if (!mScene)
  {
    sLogF(L"PhysX", L"CreateScene - createScene failed!\n");
    return sFALSE;
  }

  return sTRUE;
}

void RNPhysx::BuildActor(WpxActor * wa)
{
  // get pointer to rigidmesh to retrieve actors data of this input
  PXRigidMesh * act = (PXRigidMesh*)wa->RootNode;

  sVERIFY(act);

  // add all actors of this input to the global data list
  AllActorData.Add(act->ActorData);

  // add joints if exists
  if(act->Joints && act->Joints->GetCount() > 0)
    Joints.Add(*act->Joints);

  // for each actors of this input
  for(sInt i=0; i<act->ActorData.GetCount(); i++)
  {
    // array of shapes densities to compute global mass and inertia of current actor
    sArray<PxReal> shapeDensities;

    // get global actor pose
    PxMat44 pxActorMat;
    sMatrix34ToPxMat44(*act->ActorData[i].GlobalMatrixPtr, pxActorMat);
    PxTransform pose(pxActorMat);

    // local ptr to rigid body
    PxRigidDynamic * rigidDynamic = 0;
    PxRigidStatic * rigidStatic = 0;

    // dynamic or static ?
    switch(act->ActorData[i].ParaPtr->ActorType)
    {
      case EAT_STATIC:
        rigidStatic = gPhysicsSDK->createRigidStatic(PxTransform::createIdentity());  
        break;

      case EAT_DYNAMIC:
        rigidDynamic = gPhysicsSDK->createRigidDynamic(PxTransform::createIdentity());

        // kinematic ?
        if(act->ActorData[i].ParaPtr->DynamicType == 1)
        {
          // activate kinematic
          rigidDynamic->setRigidDynamicFlag(PxRigidDynamicFlag::eKINEMATIC, true);

          // add this actor to kinematic actor list, (to be accessible in PXRigidMesh for script animation)
          sVERIFY(act->PtrKinematicActorList);
          sRigidDynamicActor ka;
          ka.RigidDynamicPtr = rigidDynamic;
          ka.ActorPtr = &act->ActorData[i];
          act->PtrKinematicActorList->AddTail(ka);
        }
        else
        {
          // register this actor for animation
          if(act->ActorData[i].ParaPtr->TimeFlag == 1)
          {
            // add this actor to animated actor list
            sRigidDynamicActor ra;
            ra.ActorPtr = &act->ActorData[i];
            ra.RigidDynamicPtr = rigidDynamic;
            act->PtrAnimatedActorList->AddTail(ra);
          }
        }
        break;
    }

    // get colliders ptr for this actors
    sColliderObj * colliderObj = act->ActorData[i].ColliderDataPtr->GetData();

    // for each colliders shapes of current actor
    for(sInt j=0; j<act->ActorData[i].ColliderDataPtr->GetCount(); j++)
    {
      sVERIFY(colliderObj);

      // create physx material
      PxMaterial* material = gPhysicsSDK->createMaterial(colliderObj->ParaPtr->StaticFriction, colliderObj->ParaPtr->DynamicFriction, colliderObj->ParaPtr->Restitution);

      // store in array the shape density (to update final mass and inertia when all shapes will be created)
      shapeDensities.AddTail(colliderObj->ParaPtr->Density);

      // create geometry according type
      PxGeometry * geometry = 0;
      switch(colliderObj->ParaPtr->GeometryType)
      {
      case EGT_CUBE:
        {
          PxVec3 dimensions(colliderObj->ParaPtr->Dimension.x/2, colliderObj->ParaPtr->Dimension.y/2, colliderObj->ParaPtr->Dimension.z/2);
          geometry = new PxBoxGeometry(dimensions);
        }
        break;

      case EGT_SPHERE:
        {
          geometry = new PxSphereGeometry(colliderObj->ParaPtr->Radius/2);
        }
        break;

      case EGT_PLANE:
        {
          geometry = new PxPlaneGeometry();
        }
        break;

      case EGT_HULL:
        {
          geometry = new PxConvexMeshGeometry(colliderObj->ConvexMeshPtr);
        }
        break;

      case EGT_MESH:
        {
          geometry = new PxTriangleMeshGeometry(colliderObj->TriMeshPtr);
        }
        break;
      }

      sVERIFY(geometry!=0);

      // apply shape transformation
      sMatrix34 wzMat34;
      sSRT srt;
      srt.Scale = sVector31(1.0f); //rbi[i]->Scale;
      srt.Rotate = colliderObj->ParaPtr->Rot;
      if(colliderObj->ParaPtr->GeometryType == EGT_PLANE)
        srt.Rotate.z += 0.25;
      srt.Translate = colliderObj->ParaPtr->Trans;
      srt.MakeMatrix(wzMat34);      
      PxMat44 pxMat;
      sMatrix34ToPxMat44(wzMat34, pxMat);
      PxTransform transform(pxMat);

      // create a colliding shape for this actor
      switch(act->ActorData[i].ParaPtr->ActorType)
      {
        case EAT_STATIC:
          rigidStatic->createShape(*geometry, *material, transform);
          break;

        case EAT_DYNAMIC:
          PxShape * p = rigidDynamic->createShape(*geometry, *material, transform);
          //PxShape * p = wa->ActorData[i]->rigidDynamic->createShape(*geometry, *material, transform);
          //p->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
          break;
      }

      // delete no more used data
      delete geometry;
      material->release();

      // go next collider for this actor
      colliderObj++;
    }
    
    // now, all colliders shape have been created for this actor

    // final settings for this actor
    switch(act->ActorData[i].ParaPtr->ActorType)
    {
      case EAT_STATIC:
        rigidStatic->setGlobalPose(pose);
        mScene->addActor(*rigidStatic);
        break;

      case EAT_DYNAMIC:        
        mScene->addActor(*rigidDynamic);

        // pose
        rigidDynamic->setGlobalPose(pose);

        // kinematic is not concerned
        if(act->ActorData[i].ParaPtr->DynamicType != 1)
        {         
          // get a para pointer for easiest code 
          WpxActorParaRigidBody  * para = act->ActorData[i].ParaPtr;          

          // sleep threshold
          rigidDynamic->setSleepThreshold(para->SleepThreshold);          
          
          // auto MassAndInertia ?
          if(para->MassAndInertia == 0)
          {
            // auto compute mass and inertia according all shapes colliders densities
            PxReal * sd = shapeDensities.GetData();   
            PxRigidBodyExt::updateMassAndInertia(*rigidDynamic, sd, shapeDensities.GetCount());            
          }
          else
          {
            // manual set of mass and center of mass
            PxReal mass = para->Mass;
            PxVec3 massLocalPose(para->CenterOfMass.x, para->CenterOfMass.y, para->CenterOfMass.z);
            PxRigidBodyExt::setMassAndUpdateInertia(*rigidDynamic,mass,&massLocalPose);
          }

          // linear velocity
          PxVec3 linearVelocity(para->LinearVelocity.x, para->LinearVelocity.y, para->LinearVelocity.z);
          rigidDynamic->setLinearVelocity(linearVelocity);
          
          // angular velocity
          PxReal maxAngVel = para->MaxAngularVelocity;
          rigidDynamic->setMaxAngularVelocity(maxAngVel);
          PxVec3 angVel(para->AngularVelocity.x, para->AngularVelocity.y, para->AngularVelocity.z);
          rigidDynamic->setAngularVelocity(angVel);

          // gravity flag
          bool gravityFlag = !para->Gravity;
          rigidDynamic->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, gravityFlag); 

          // damping 
          PxReal linDamp = para->LinearDamping;
          rigidDynamic->setLinearDamping(linDamp);
          PxReal angDamp = para->AngularDamping;
          rigidDynamic->setAngularDamping(angDamp);

          // force
          if(para->ForceMode != 4)
          {
            PxVec3 force(para->Force.x, para->Force.y, para->Force.z);
            PxForceMode::Enum forceMode = (PxForceMode::Enum)para->ForceMode;
            rigidDynamic->addForce(force, forceMode);
          }

          // torque
          if(para->TorqueMode != 4)
          {
            PxVec3 torque(para->Torque.x, para->Torque.y, para->Torque.z);
            PxForceMode::Enum torqueMode = (PxForceMode::Enum)para->TorqueMode;
            rigidDynamic->addTorque(torque, torqueMode);
          }

          // sleep ?
          if(para->Sleep)
            rigidDynamic->putToSleep();

        }
        break;
    }

    // clear shape densities array for next actor
    shapeDensities.Clear();   
  }
}

void RNPhysx::Init()
{
  // init time variables
  LastTime = 0.0f;
  Accumulator = 0.0f;
  PreviousTimeLine = 0.0f;

  // init actors buffer
  PxActorTypeSelectionFlags desiredTypes = PxActorTypeSelectionFlag::eRIGID_DYNAMIC | PxActorTypeSelectionFlag::eRIGID_STATIC;
  ActorCount = mScene->getNbActors(desiredTypes);
  ActorBuffer = new PxActor*[ActorCount];
  mScene->getActors(desiredTypes, ActorBuffer, ActorCount);

  Executed = sFALSE;
}

void RNPhysx::Simulate(Wz4RenderContext *ctx)
{  
  gCumulatedCount = 0;
  
  Para = ParaBase; 
  Anim.Bind(ctx->Script,&Para);
  SimulateCalc(ctx);
 // SimulateChilds(ctx);
  
  if(!Para.Enable)
    return;

  if(!mScene)
    return;


  // restart simulation mechanism (press F6)

  if(!Doc->IsPlayer)
  {
    // compute elpased wz time for the restart mechanism 
    sF32 timeLine = ctx->GetBaseTime();
    sF32 deltaTimeLine = timeLine - PreviousTimeLine;  
    PreviousTimeLine = timeLine;

    // timeline is paused, no simulation to run
    if(deltaTimeLine == 0)
      return;

    // Restart simul mechanism (for F6, loop demo, or clip restart)
    if(Executed && deltaTimeLine < -0.1f)    
    {
      // rebuild op
      Doc->Change(Op,1,1);
      Executed = sFALSE;
      return;
    }
    Executed = sTRUE;
  }  
  
  // simulation loop

  sF32 timeStep = 1.0f / sMax(10,Para.TimeStep);

  // avoid negative value if GetBaseTime goes back
  if(Accumulator < 0)
    Accumulator = 0;

  // compute real elapsed time to synchronize simulation with real time
  sF32 newTime = sGetTimeUS() * 0.001;
  sF32 deltaTime = newTime - LastTime;
  LastTime = newTime;

  Accumulator += deltaTime;
  if (Accumulator > 10*timeStep) Accumulator = timeStep;

  while (Accumulator >= timeStep)
  {
    Accumulator -= timeStep;

    // count nb time to cumulate forces for animated rigid dynamics
    gCumulatedCount++;

    mScene->simulate(timeStep);
    mScene->fetchResults(Para.WaitFetchResults);
  }

  SimulateChilds(ctx);
}

void RNPhysx::Transform(Wz4RenderContext *ctx,const sMatrix34 & mat)
{
  MatrixTransform = mat;
}

void RNPhysx::Prepare(Wz4RenderContext *ctx) 
{  
  PxTransform pT;
  sMatrix34 mmat;
  MatricesActors.Clear();

  // for each actor
  for(sInt i=0; i<AllActorData.GetCount();)
  {
    sActorMatrices am;
   
    // for each copy of this actor
    for(sInt j=0; j<AllActorData[i].InstanceCount; j++)
    {
      // compute matrix and add it to an array
      pT = static_cast<PxRigidActor*>(ActorBuffer[i+j])->getGlobalPose();
      PxMat44TosMatrix34(pT, mmat);
    
      // add matrice in array for this actor
      am.Matrices.AddTail(sMatrix34CM(mmat*MatrixTransform));
    }
    
    // associate mesh ptr to matrices array
    am.MeshPtr = AllActorData[i].MeshPtr;
    MatricesActors.AddTail(am);
    
    // prepare mesh according matrix array
    if(AllActorData[i].MeshPtr)
      AllActorData[i].MeshPtr->BeforeFrame(Para.EnvNum,am.Matrices.GetCount(),am.Matrices.GetData());   

    // advance to next unique actor
    i+=AllActorData[i].InstanceCount;
  }
}

void RNPhysx::Render(Wz4RenderContext *ctx)
{
  for(sInt i=0; i<MatricesActors.GetCount(); i++)
  {
    if(MatricesActors[i].MeshPtr)
      MatricesActors[i].MeshPtr->RenderInst(ctx->RenderMode,Para.EnvNum, MatricesActors[i].Matrices.GetCount(), MatricesActors[i].Matrices.GetData());
  }
}


void SetFixedJoint(PxJoint * joint, sJoint * j)
{
  PxFixedJoint * fixedJoint = static_cast<PxFixedJoint *>(joint);

  if(j->Para->FixedProjectionFlag)
  { 
    fixedJoint->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
    fixedJoint->setProjectionLinearTolerance(j->Para->FixedProjectionLinearTolerance);
    fixedJoint->setProjectionAngularTolerance(sDEG2RAD(j->Para->FixedProjectionAngularTolerance));    
  }
}

void SetSphericalJoint(PxJoint * joint, sJoint * j)
{
  PxSphericalJoint * sphericalJoint = static_cast<PxSphericalJoint *>(joint);
  
  if(j->Para->LimitConeFlag)
  {    
    sphericalJoint->setSphericalJointFlag(PxSphericalJointFlag::eLIMIT_ENABLED, sTRUE);
    
    PxReal a,b,c;
    a = sDEG2RAD(j->Para->LimitConeYLimitAngle);
    b = sDEG2RAD(j->Para->LimitConeZLimitAngle);
    c = j->Para->LimitConeLimitContactDistance;

    PxJointLimitCone limit(a,b,c);
    limit.damping = j->Para->SphericalLimitDamping;
    limit.restitution = j->Para->SphericalLimitRestitution;
    limit.spring = j->Para->SphericalLimitSpring;

    sphericalJoint->setLimitCone(limit);
  }

  if(j->Para->SphericalProjectionFlag)
  { 
    sphericalJoint->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
    sphericalJoint->setProjectionLinearTolerance(j->Para->SphericalProjectionLinearTolerance);   
  }
}

void SetRevoluteJoint(PxJoint * joint, sJoint * j)
{
  PxRevoluteJoint * revoluteJoint = static_cast<PxRevoluteJoint *>(joint);

  if(j->Para->LimitPrismaticFlag)
  {
    revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eLIMIT_ENABLED, sTRUE);

    PxReal a,b,c;
    a = sDEG2RAD(j->Para->RevoluteLowerLimit);
    b = sDEG2RAD(j->Para->RevoluteUpperLimit);
    c = j->Para->RevoluteLimitContactDistance;   

    PxJointLimitPair limit(a,b,c);
    limit.damping = j->Para->RevoluteLimitDamping;
    limit.restitution = j->Para->RevoluteLimitRestitution;
    limit.spring = j->Para->RevoluteLimitSpring;

    revoluteJoint->setLimit(limit);
  }

  if(j->Para->RevoluteDriveEnabled)
    revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_ENABLED, sTRUE);

  if(j->Para->RevoluteFreeSpinEnabled)
    revoluteJoint->setRevoluteJointFlag(PxRevoluteJointFlag::eDRIVE_FREESPIN, true);

  revoluteJoint->setDriveForceLimit(j->Para->DriveForceLimit);
  revoluteJoint->setDriveGearRatio(j->Para->DriveGearRatio);
  revoluteJoint->setDriveVelocity(j->Para->DriveVelocity);

  if(j->Para->RevoluteProjectionFlag)
  { 
    revoluteJoint->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
    revoluteJoint->setProjectionLinearTolerance(j->Para->RevoluteProjectionLinearTolerance);
    revoluteJoint->setProjectionAngularTolerance(sDEG2RAD(j->Para->RevoluteProjectionAngularTolerance));    
  }
}

void SetPrismaticJoint(PxJoint * joint, sJoint * j)
{
  PxPrismaticJoint * prismaticJoint = static_cast<PxPrismaticJoint *>(joint);

  if(j->Para->LimitPrismaticFlag)
  {
    prismaticJoint->setPrismaticJointFlag(PxPrismaticJointFlag::eLIMIT_ENABLED, sTRUE);
    
    PxReal a,b,c;
    a = sDEG2RAD(j->Para->PrismaticLowerLimit);
    b = sDEG2RAD(j->Para->PrismaticUpperLimit);
    c = j->Para->PrismaticLimitContactDistance;

    PxJointLimitPair limit(a,b,c);    
    limit.damping = j->Para->PrismaticLimitDamping;
    limit.restitution = j->Para->PrismaticLimitRestitution;
    limit.spring = j->Para->PrismaticLimitSpring;

    prismaticJoint->setLimit(limit);
  }

  if(j->Para->PrismaticProjectionFlag)
  { 
    prismaticJoint->setConstraintFlag(PxConstraintFlag::ePROJECTION, true);
    prismaticJoint->setProjectionLinearTolerance(j->Para->PrismaticProjectionLinearTolerance);
    prismaticJoint->setProjectionAngularTolerance(sDEG2RAD(j->Para->PrismaticProjectionAngularTolerance));    
  }
}

void SetDistanceJoint(PxJoint * joint, sJoint * j)
{
  PxDistanceJoint * distanceJoint = static_cast<PxDistanceJoint *>(joint);

  if(j->Para->MaxDistanceEnable)
  {
    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMAX_DISTANCE_ENABLED, sTRUE);
    distanceJoint->setMaxDistance(j->Para->DistanceMax);
  }

  if(j->Para->MinDistanceEnable)
  {
    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eMIN_DISTANCE_ENABLED, sTRUE);
    distanceJoint->setMinDistance(j->Para->DistanceMin);
  }

  if(j->Para->SpringEnable)
  {
    distanceJoint->setDistanceJointFlag(PxDistanceJointFlag::eSPRING_ENABLED, sTRUE);
    distanceJoint->setSpring(j->Para->SpringStrength);
    distanceJoint->setDamping(j->Para->SpringDamping);
  }
}

void RNPhysx::CreateJoints()
{
  sJoint * j;  
  sFORALL(Joints, j)
  {
    sInt ia = sFindIndex(AllActorData, &sActorObj::ActorId, j->ActorA_ID);
    sInt ib = sFindIndex(AllActorData, &sActorObj::ActorId, j->ActorB_ID);

    if(ia != -1 && ib != -1 && ia != ib)
    { 
      PxJoint * joint = 0;
      PxRigidActor * ra1 = static_cast<PxRigidActor*>(ActorBuffer[ia]);
      PxRigidActor * ra2 = static_cast<PxRigidActor*>(ActorBuffer[ib]);      

      switch(j->TypeJoint)
      {
      case 0: // fixed
        joint = PxFixedJointCreate(*gPhysicsSDK, ra1, j->ActorA_Pose, ra2, j->ActorB_Pose );
        if(joint && j->Para->FixedSettings)
            SetFixedJoint(joint, j);         
        break;

      case 1: // spherical
          joint = PxSphericalJointCreate(*gPhysicsSDK, ra1, j->ActorA_Pose, ra2, j->ActorB_Pose );
          if(joint && j->Para->SphericalSettings)
            SetSphericalJoint(joint, j);   
        break;

      case 2: // revolute
          joint = PxRevoluteJointCreate(*gPhysicsSDK, ra1, j->ActorA_Pose, ra2, j->ActorB_Pose);
          if(joint && j->Para->RevoluteSettings)
            SetRevoluteJoint(joint, j); 
        break;

      case 3: // prismatic
          joint = PxPrismaticJointCreate(*gPhysicsSDK, ra1, j->ActorA_Pose, ra2, j->ActorB_Pose );
          if(joint && j->Para->PrismaticSettings)
            SetPrismaticJoint(joint, j);
        break;

      case 4: // distance
          joint = PxDistanceJointCreate(*gPhysicsSDK, ra1, j->ActorA_Pose, ra2, j->ActorB_Pose );
          if(joint && j->Para->DistanceSettings)
            SetDistanceJoint(joint, j);
        break;
      }

      if(joint)
      {
        // set breakable
        if(j->IsBreakable)
          joint->setBreakForce(j->Para->BreakForceMax, j->Para->BreakTorqueMax);

        // joints mesh can collide each other
        if(j->IsColliding)
          joint->setConstraintFlag(PxConstraintFlag::eCOLLISION_ENABLED, true);        
      }
    }
  }
}

/****************************************************************************/

PXActorJoint::PXActorJoint()
{
}

PXActorJoint::~PXActorJoint()
{
  Joints->Reset();
  delete Joints;
}

void PXActorJoint::Simulate(Wz4RenderContext *ctx)
{
  SimulateChilds(ctx);
}

void PXActorJoint::Init(WpxActor *a, WpxActorArrayActorJoint * array, sInt arrayCount)
{
  // add all actors
  PXRigidMesh * actors = (PXRigidMesh*)a->RootNode;
  ActorData.Add(actors->ActorData);

  // copy kinematic actor list ptr
  PtrKinematicActorList = actors->PtrKinematicActorList;

  // copy animated actor list ptr
  PtrAnimatedActorList = actors->PtrAnimatedActorList;

  // create joint array and copy existing data of eventually previous joint operator
  Joints = new sArray<sJoint>;
  if(actors->Joints)
    Joints->Add(*actors->Joints);

  // for each element create a joint
  for(sInt i=0; i<arrayCount; i++)
  {
    sJoint joint;
    
    // single/single
    if(array->ModeA == 0 && array->ModeB == 0)
    {
      sInt ia = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorA);
      sInt ib = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorB);

      if(ia != -1 && ib != -1 && ia != ib)
      {
        if(ActorData[ia].JointsPoses.IsIndexValid(array->JointA_ID) && ActorData[ib].JointsPoses.IsIndexValid(array->JointB_ID))
        {
          PxTransform ta = ActorData[ia].JointsPoses[array->JointA_ID]; 
          PxTransform tb = ActorData[ib].JointsPoses[array->JointB_ID];

          joint.ActorA_ID = array->ActorA;
          joint.ActorB_ID = array->ActorB;

          joint.ActorA_Pose = ta;
          joint.ActorB_Pose = tb;
          joint.TypeJoint = array->Type;
          joint.Para = &Para;
          joint.IsBreakable = array->Breakable;
          joint.IsColliding = array->CollideJoint;

          Joints->AddTail(joint);
        }
      }    
    }

    // single/range
    if(array->ModeA == 0 && array->ModeB == 1)
    {
      sInt ia = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorA);

      for(sInt x=0; x<array->CountB; x++)
      {
        sInt ib = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorB + x);

        if(ia != -1 && ib != -1 && ia != ib)
        {
          if(ActorData[ia].JointsPoses.IsIndexValid(array->JointA_ID) && ActorData[ib].JointsPoses.IsIndexValid(array->JointB_ID))
          {
            PxTransform ta = ActorData[ia].JointsPoses[array->JointA_ID]; 
            PxTransform tb = ActorData[ib].JointsPoses[array->JointB_ID];

            joint.ActorA_ID = array->ActorA;
            joint.ActorB_ID = array->ActorB + x;

            joint.ActorA_Pose = ta;
            joint.ActorB_Pose = tb;
            joint.TypeJoint = array->Type;
            joint.Para = &Para;
            joint.IsBreakable = array->Breakable;
            joint.IsColliding = array->CollideJoint;

            Joints->AddTail(joint);
          }
        }
      }    
    }

    // range/single
    if(array->ModeA == 1 && array->ModeB == 0)
    {
      sInt ib = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorB);

      for(sInt x=0; x<array->CountA; x++)
      {
        sInt ia = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorA + x);

        if(ia != -1 && ib != -1 && ia != ib)
        {
          if(ActorData[ia].JointsPoses.IsIndexValid(array->JointA_ID) && ActorData[ib].JointsPoses.IsIndexValid(array->JointB_ID))
          {
            PxTransform ta = ActorData[ia].JointsPoses[array->JointA_ID]; 
            PxTransform tb = ActorData[ib].JointsPoses[array->JointB_ID];

            joint.ActorA_ID = array->ActorA + x;
            joint.ActorB_ID = array->ActorB;

            joint.ActorA_Pose = ta;
            joint.ActorB_Pose = tb;
            joint.TypeJoint = array->Type;
            joint.Para = &Para;
            joint.IsBreakable = array->Breakable;
            joint.IsColliding = array->CollideJoint;

            Joints->AddTail(joint);
          }
        }
      }    
    }

    // range/range
    if(array->ModeA == 1 && array->ModeB == 1)
    {
      for(sInt y=0; y<sMin(array->CountA, array->CountB); y++)
      {
        sInt ia = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorA + y);
        sInt ib = sFindIndex(ActorData, &sActorObj::ActorId, array->ActorB + y);

        if(ia != -1 && ib != -1 && ia != ib)
        {
          if(ActorData[ia].JointsPoses.IsIndexValid(array->JointA_ID) && ActorData[ib].JointsPoses.IsIndexValid(array->JointB_ID))
          {
            PxTransform ta = ActorData[ia].JointsPoses[array->JointA_ID]; 
            PxTransform tb = ActorData[ib].JointsPoses[array->JointB_ID];

            joint.ActorA_ID = array->ActorA + y;
            joint.ActorB_ID = array->ActorB + y;

            joint.ActorA_Pose = ta;
            joint.ActorB_Pose = tb;
            joint.TypeJoint = array->Type;
            joint.Para = &Para;
            joint.IsBreakable = array->Breakable;
            joint.IsColliding = array->CollideJoint;

            Joints->AddTail(joint);
          }
        }
      }    
    }

    array++;
  }
}