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

#ifdef _DEBUG
#undef _DEBUG
#define _DEBUG_WAS_DEFINED
#endif

#undef new
#include "C:/library/PhysX-3.2.3_PC_SDK_Core/Include/PxPhysicsAPI.h"
#define new sDEFINE_NEW

#ifdef _DEBUG_WAS_DEFINED
#undef _DEBUG_WAS_DEFINED
#define _DEBUG
#endif

#ifdef _M_X64
// 64 bits
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win64/PhysX3CHECKED_x64.lib")
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win64/PhysX3CommonCHECKED_x64.lib")
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win64/PhysX3ExtensionsCHECKED.lib")
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win64/PhysX3CookingCHECKED_x64.lib")
#else
// 32 bits
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win32/PhysX3CHECKED_x86.lib")
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win32/PhysX3CommonCHECKED_x86.lib")
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win32/PhysX3ExtensionsCHECKED.lib")
#pragma comment(lib, "C:/library/PhysX-3.2.3_PC_SDK_Core/Lib/win32/PhysX3CookingCHECKED_x86.lib")
#endif

#ifdef _DEBUG
#pragma comment(linker, "/NODEFAULTLIB:libcmt.lib")
#endif

using namespace physx;

/****************************************************************************/
/****************************************************************************/

extern PxFoundation * gFoundation;
extern PxPhysics * gPhysicsSDK;
extern PxCooking * gCooking;
void PhysXInitEngine();

/****************************************************************************/
/****************************************************************************/

class MemoryOutputStream: public PxOutputStream
{
public:
  MemoryOutputStream();
  virtual ~MemoryOutputStream();
  PxU32 write(const void* src, PxU32 count);
  PxU32 getSize()	const	{	return mSize; }
  PxU8* getData()	const	{	return mData; }

private:
  PxU8* mData;
  PxU32 mSize;
  PxU32 mCapacity;
};

/****************************************************************************/

class MemoryInputData: public PxInputData
{
public:
  MemoryInputData(PxU8* data, PxU32 length);
  PxU32 read(void* dest, PxU32 count);
  PxU32 getLength() const;
  void seek(PxU32 pos);
  PxU32 tell() const;

private:
  PxU32 mSize;
  const PxU8*	mData;
  PxU32 mPos;
};

/****************************************************************************/
/****************************************************************************/

enum E_GEOMETRY_TYPE
{  
  EGT_CUBE = 0,
  EGT_SPHERE,
  EGT_PLANE,      // physx don't support plane colliding with another plane
  EGT_HULL,       // convex hull mesh is limited to 256 polygons
  EGT_MESH,       // physx don't support mesh colliding with another mesh
  EGT_NONE
};

/****************************************************************************/

enum E_ACTOR_TYPE
{
  EAT_STATIC = 0,
  EAT_DYNAMIC
};

/****************************************************************************/

struct sColliderObj
{
public:

  sColliderObj():
      ShapePtr(0),
      ConvexMeshPtr(0),
      TriMeshPtr(0),
      ParaPtr(0)
  {}

  Wz4Mesh * ShapePtr;                     // shape mesh ptr  
  PxConvexMesh * ConvexMeshPtr;           // convex mesh ptr (when GeometryType is hull)
  PxTriangleMesh * TriMeshPtr;            // triangle mesh ptr (when GeometryType is mesh)
  WpxColliderParaShapeCollider * ParaPtr; // op parameters ptr
};

/****************************************************************************/

struct sActorObj
{
public:

  sActorObj() :
    MeshPtr(0),
    ParaPtr(0),
    GlobalMatrixPtr(0),
    ColliderDataPtr(0),
    InstanceCount(1),
    JointMeshPtr(0),
    JointsMatrixArrayPtr(0)
  {}

  sMatrix34 * GlobalMatrixPtr;               // actor matrix ptr
  WpxActorParaRigidBody * ParaPtr;           // actor operator parameter ptr
  Wz4Mesh * MeshPtr;                         // rendered mesh actor ptr
  sArray<sColliderObj> * ColliderDataPtr;    // ptr to colliders data
  sInt InstanceCount;                        // nbr of instance for this actor

  // specifics to joints
  Wz4Mesh * JointMeshPtr;                     // joint mesh ptr used to preview joint pose when show operator
  sArray<sMatrix34 *> * JointsMatrixArrayPtr; // joint matrix array ptr used to paint each joint poses for current actor
  sInt ActorId;                               // current actor ID
  sArray<PxTransform> JointsPoses;            // joints poses for current actor
};

/****************************************************************************/

struct sJoint
{
  sJoint() :
    ActorA_ID(-1),
    ActorB_ID(-1)
  {}

  sInt TypeJoint;
  sInt ActorA_ID;
  sInt ActorB_ID;
  PxTransform ActorA_Pose;
  PxTransform ActorB_Pose;
  WpxActorParaActorJoint * Para;
  sBool IsBreakable;
  sBool IsColliding;
};

/****************************************************************************/

struct sRigidDynamicActor
{
  PxRigidDynamic * RigidDynamicPtr;
  sActorObj * ActorPtr;
};

/****************************************************************************/
/****************************************************************************/

class WpxCollider : public Wz4Render
{
public:  
  sArray<sColliderObj> * ColliderDataPtr;   // ptr to list of colliders data

  WpxCollider() { Type = WpxColliderType; }
};

/****************************************************************************/

class WpxActor : public Wz4Render
{
public:  
  sArray<sActorObj> * ActorDataPtr;         // ptr to list of actor data

  WpxActor() { Type = WpxActorType; }
};
  
/****************************************************************************/
/****************************************************************************/

class PXShapeCollider : public Wz4RenderNode
{
public:
  sArray<sColliderObj> ColliderData;    // list of colliders data
  Wz4Mesh * ShapeMesh;                  // shapemesh, used to render collider mesh
  PxConvexMesh * ConvexMesh;            // convexmesh
  PxTriangleMesh  * TriMesh;            // trimesh

  WpxColliderParaShapeCollider ParaBase,Para;
  WpxColliderAnimShapeCollider Anim;

  PXShapeCollider();
  ~PXShapeCollider();
  void Init(Wz4Mesh * inMesh);
};

/****************************************************************************/

class PXShapeAdd : public PXShapeCollider
{
public:
  WpxColliderParaShapeAdd ParaBase,Para;
  WpxColliderAnimShapeAdd Anim;

  PXShapeAdd();
  ~PXShapeAdd();
  void AddCollider(WpxCollider* colOp);
};

/****************************************************************************/

class PXRigidMesh : public  Wz4RenderNode
{
private:
  sArray<sRigidDynamicActor> KinematicActorList;    // list of rigiddynamic kinematic actors
  sArray<sRigidDynamicActor> AnimatedActorPtrList;  // list of rigiddynamic actors registered for animation

  // for vertices build mode
  sArray<sMatrix34 *>  CreatedMatArray;             // store ptr of all created new matrix for clean memory deletion at destruction
  sArray<Wz4Mesh *>   CreatedMeshArray;             // store ptr of all created new mesh for clean memory deletion at destruction

  // joints relative
  sArray<sMatrix34*> JointsMatrix;                  // all joint poses for this actor    
  Wz4Mesh * JointMesh;                              // mesh to represent joint pose

  void CreateJointMeshViewer();

public:
  sMatrix34 Matrix;                                     // actor matrix
  sArray<sActorObj> ActorData;                          // list of actor data
  sArray<sRigidDynamicActor> * PtrKinematicActorList;   // ptr to list of kinematics actors, used for animation
  sArray<sRigidDynamicActor> * PtrAnimatedActorList;    // ptr to list of dynamics actors, used for animation
  sArray<sJoint> * Joints;

  WpxActorParaRigidBody ParaBase,Para;
  WpxActorAnimRigidBody Anim;

  PXRigidMesh();
  ~PXRigidMesh();  
  void Transform(Wz4RenderContext *ctx,const sMatrix34 &mat);
  void Simulate(Wz4RenderContext *ctx);
  void BuildRigidBodyFromMesh(WpxCollider * in, Wz4Mesh * mesh);
  void BuildRigidBodyFromVertices(WpxCollider * in, Wz4Mesh * mesh, Wz4Mesh * meshModel);
  void BuildJoints(WpxActorArrayRigidBody * array, sInt arrayCount);  
};

/****************************************************************************/

class PXActorAdd : public PXRigidMesh
{
public:  
  WpxActorParaActorAdd ParaBase,Para;
  WpxActorAnimActorAdd Anim;

  PXActorAdd();
  ~PXActorAdd();
  void Simulate(Wz4RenderContext *ctx);
  void AddActor(WpxActor * actOp);
};

/****************************************************************************/

class PXActorJoint : public PXRigidMesh
{
public:
  WpxActorParaActorJoint ParaBase,Para;
  WpxActorAnimActorJoint Anim;

  PXActorJoint();
  ~PXActorJoint();  
  void Init(WpxActor *a, WpxActorArrayActorJoint * array, sInt arrayCount);
  void Simulate(Wz4RenderContext *ctx);
};

/****************************************************************************/

class PXActorMultiply : public PXRigidMesh
{
public:
  sArray<sMatrix34  *> MulMatArray;     // store ptr of all created new mul matrix for clean memory deletion at destruction
  sArray<sActorObj *> ActorDataArray;   // store ptr of all created new sActorObj for clean memory deletion at destruction

  WpxActorParaActorMultiply ParaBase,Para;
  WpxActorAnimActorMultiply Anim;  

  PXActorMultiply();
  ~PXActorMultiply();
  void Simulate(Wz4RenderContext *ctx);
};

/****************************************************************************/

struct sActorMatrices
{
  Wz4Mesh * MeshPtr;
  sArray<sMatrix34CM> Matrices;
};

class RNPhysx : public Wz4RenderNode
{
private:
  sBool Executed;                         // flag for restart simulation mecanisme with F6
  sF32 LastTime;                          // last frame time
  sF32 Accumulator;                       // time accumulator
  sF32 PreviousTimeLine;                  // delta time line use to restart simulation
  PxActor** ActorBuffer;                  // buffer of actors in the scene
  PxU32 ActorCount;                       // nb actor in buffer
  sMatrix34 MatrixTransform;              // matrix computed in transform() and used to transform scene node by others ops
  PxScene * mScene;                       // physx scene
  sArray<sActorObj> AllActorData;         // all actors Data
  sArray<sActorMatrices> MatricesActors;  // Matrices of each instanciated mesh, filled in Prepare() and used in Render()
  sArray<sJoint> Joints;                  // list of all joints  

public:
  Wz4RenderParaPhysx ParaBase,Para;
  Wz4RenderAnimPhysx Anim;

  RNPhysx();
  ~RNPhysx();
  sBool CreateScene();
  void CreateJoints();
  void BuildActor(WpxActor * wa);
  void Init();
  void Simulate(Wz4RenderContext *ctx);
  void Transform(Wz4RenderContext *ctx,const sMatrix34 & mat);
  void Prepare(Wz4RenderContext *ctx);
  void Render(Wz4RenderContext *ctx);  
};

/****************************************************************************/

#endif FILE_WZ4FRLIB_PHYSX_HPP