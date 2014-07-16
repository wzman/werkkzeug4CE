/*+**************************************************************************/
/***                                                                      ***/
/***   This file is distributed under a BSD license.                      ***/
/***   See LICENSE.txt for details.                                       ***/
/***                                                                      ***/
/**************************************************************************+*/

#include "wz4_anim.hpp"
#include "wz4_anim_ops.hpp"
#include "wz4lib/serials.hpp"

/****************************************************************************/
/***                                                                      ***/
/***   Helpers                                                            ***/
/***                                                                      ***/
/****************************************************************************/

void Wz4AnimKey::Init()
{
  sClear(*this);
  Scale.Init(1,1,1);
}

void Wz4AnimKey::ToMatrix(sMatrix34 &mat) const
{
  mat.Init(Rot);
  mat.Scale(Scale.x,Scale.y,Scale.z);
  mat.l = Trans;
}

void Wz4AnimKey::ToMatrixInv(sMatrix34 &mat) const // this can be done more efficient!
{
  mat.Init(Rot);
  mat.Scale(Scale.x,Scale.y,Scale.z);
  mat.l = Trans;
  mat.InvertOrthogonal();
}

template <class streamer> void Wz4AnimKey::Serialize_(streamer &stream)
{
  sInt version = stream.Header(sSerId::Wz4AnimKey,1);
  if(version)
  {
    stream | Time | Weight | Scale | Rot | Trans | User;
    stream.Footer();
  }
}
void Wz4AnimKey::Serialize(sWriter &stream) { Serialize_(stream); }
void Wz4AnimKey::Serialize(sReader &stream) { Serialize_(stream); }

void Wz4AnimJoint::Init()
{
  Channel = 0;
  BasePose.Init();
  Parent = -1;
}

void SerializeWz4Channel(sReader &stream,Wz4Channel *&chan)
{
  switch(stream.PeekHeader())
  {
  case sSerId::Wz4ChannelPerFrame:
    { Wz4ChannelPerFrame *c = new Wz4ChannelPerFrame; c->Serialize(stream); chan=c; }
    break;
  case sSerId::Wz4ChannelSpline:
    { Wz4ChannelSpline *c = new Wz4ChannelSpline; c->Serialize(stream); chan = c; }
    break;
  default:
    sVERIFY(L"unknown channel format");
    break;
  }
}

void SerializeWz4Channel(sWriter &stream,Wz4Channel *chan)
{
  chan->Serialize(stream);
}

/****************************************************************************/
/***                                                                      ***/
/***   Various Channels                                                   ***/
/***                                                                      ***/
/****************************************************************************/

Wz4Channel::Wz4Channel()
{
  Type = Wz4ChannelType;
  Kind = Wz4ChannelKindIllegal;
  MaxTime = 1;
}

/****************************************************************************/

Wz4ChannelConstant::Wz4ChannelConstant()
{
  Start.Init();
  Kind = Wz4ChannelKindConstant;
}

void Wz4ChannelConstant::Evaluate(sF32 time,Wz4AnimKey &result)
{
  result = Start;
  result.Time = time;
}

Wz4Channel *Wz4ChannelConstant::CopyTo()
{
  Wz4ChannelConstant *ch = new Wz4ChannelConstant;
  ch->Start = Start;
  return ch;
}

/****************************************************************************/

Wz4ChannelLinear::Wz4ChannelLinear()
{
  Start.Init();
  Speed.Init(0,0,0);
  Gravity.Init(0,0,0);
  ScaleSpeed.Init(0,0,0);
  RotAxis.Init(0,1,0);
  RotSpeed = 0;
  UserStart.Init(0,0,0);
  UserSpeed.Init(0,0,0);
  Weight = 0;
  Kind = Wz4ChannelKindLinear;

  prepared = 0;
}

void Wz4ChannelLinear::Prepare()
{
  sMatrix34 mat = Start;
  mat.Trans3();
  ScaleStart.x = Start.i.Length();
  ScaleStart.y = Start.j.Length();
  ScaleStart.z = Start.k.Length();
  mat.i.Unit();
  mat.j.Unit();
  mat.k.Unit();
  mat.Trans3();
  RotStart.Init(mat);
}

void Wz4ChannelLinear::Evaluate(sF32 time,Wz4AnimKey &result)
{
  sF32 f = time/MaxTime;
  result.Time = time;
  result.Weight = Weight;
  result.pad = 0;
  result.Scale = ScaleStart + ScaleSpeed*f;
  if(RotSpeed)
  {
    sQuaternion a;
    a.Init(RotAxis,RotSpeed*f*sPI2F);
    result.Rot = RotStart * a;
  }
  else
  {
    result.Rot = RotStart;
  }
  result.Trans = Start.l + Speed*f + Gravity*f*f;
  result.User = UserStart + UserSpeed*f;
}

/****************************************************************************/

Wz4ChannelPerFrame::Wz4ChannelPerFrame()
{
  Start.Init();
  Kind = Wz4ChannelKindPerFrame;
  Keys = 0;
  Scale = 0;
  Rot = 0;
  Trans = 0;
  User = 0;
}

Wz4ChannelPerFrame::~Wz4ChannelPerFrame()
{
  delete[] Scale;
  delete[] Rot;
  delete[] Trans;
  delete[] User;
}

template <class streamer> void Wz4ChannelPerFrame::Serialize_(streamer &stream)
{
  sInt version = stream.Header(sSerId::Wz4ChannelPerFrame,1);
  if(version)
  {
    Start.Serialize(stream);
    stream | Keys;
    if(stream.IsWriting())
    {
      stream.If(Scale!=0);
      stream.If(Rot!=0);
      stream.If(Trans!=0);
      stream.If(User!=0);
    }
    else // reading
    {
      if(stream.If(0)) Scale = new sVector31[Keys];
      if(stream.If(0)) Rot   = new sQuaternion[Keys];
      if(stream.If(0)) Trans = new sVector31[Keys];
      if(stream.If(0)) User  = new sVector31[Keys];
    }

    if(Scale) stream.ArrayF32((sF32 *)Scale,Keys*3);
    if(Rot  ) stream.ArrayF32((sF32 *)Rot  ,Keys*4);
    if(Trans) stream.ArrayF32((sF32 *)Trans,Keys*3);
    if(User ) stream.ArrayF32((sF32 *)User ,Keys*3);

    stream.Footer();
  }
}
void Wz4ChannelPerFrame::Serialize(sWriter &stream) { Serialize_(stream); }
void Wz4ChannelPerFrame::Serialize(sReader &stream) { Serialize_(stream); }

Wz4Channel *Wz4ChannelPerFrame::CopyTo()
{
  Wz4ChannelPerFrame *dest = new Wz4ChannelPerFrame;
  dest->MaxTime = MaxTime;
  dest->Start = Start;
  dest->Keys = Keys;
  dest->Scale = 0;
  dest->Rot = 0;
  dest->Trans = 0;
  dest->User = 0;

  if(Scale)
  {
    dest->Scale = new sVector31[Keys];
    sCopyMem(dest->Scale,Scale,sizeof(sVector31)*Keys);
  }
  if(Rot)
  {
    dest->Rot   = new sQuaternion[Keys];
    sCopyMem(dest->Rot  ,Rot  ,sizeof(sQuaternion)*Keys);
  }
  if(Trans)
  {
    dest->Trans = new sVector31[Keys];
    sCopyMem(dest->Trans,Trans,sizeof(sVector31)*Keys);
  }
  if(User)
  {
    dest->User  = new sVector31[Keys];
    sCopyMem(dest->User ,User ,sizeof(sVector31)*Keys);
  }

  return dest;
}

void Wz4ChannelPerFrame::Evaluate(sF32 time,Wz4AnimKey &result)
{
  result = Start;
  if(Keys>1)
  {
    time = (time / MaxTime) * (Keys-1);
    sInt key = sClamp<sInt>(sInt(sRoundDown(time*1024)),0,Keys*1024-1025);
    sF32 f = (key&1023)/1024.0f;
    key = key/1024;

    if(Scale)
      result.Scale.Fade(f,Scale[key+0],Scale[key+1]);
    if(Rot)
      result.Rot.Fade(f,Rot[key+0],Rot[key+1]);
    if(Trans)
      result.Trans.Fade(f,Trans[key+0],Trans[key+1]);
  }
}

/****************************************************************************/

Wz4ChannelSpline::Wz4ChannelSpline()
{
  Start.Init();
  Kind = Wz4ChannelKindSpline;
}

Wz4ChannelSpline::~Wz4ChannelSpline()
{
}

template <class streamer> void Wz4ChannelSpline::Serialize_(streamer &stream)
{
  sInt version = stream.Header(sSerId::Wz4ChannelSpline,1);
  if(version)
  {
    stream | Keys;
    Start.Serialize(stream);
    Scale.Serialize(stream);
    Rot.Serialize(stream);
    Trans.Serialize(stream);

    stream.Footer();
  }
}

void Wz4ChannelSpline::Serialize(sWriter &stream) { Serialize_(stream); }
void Wz4ChannelSpline::Serialize(sReader &stream) { Serialize_(stream); }

sBool Wz4ChannelSpline::Approximate(const Wz4ChannelPerFrame& target,sF32 err)
{
  Start = target.Start;
  Keys = target.Keys;
  MaxTime = target.MaxTime;
  sBool ok = sTRUE;

  if(target.Scale)  ok &= Scale.FitToCurve((sVector30*) target.Scale,target.Keys,err,1.0f);
  if(target.Trans)  ok &= Trans.FitToCurve((sVector30*) target.Trans,target.Keys,err,1.0f);

  if(target.Rot)
  {
    // need to clean up rotation first (get rid of sign flips)
    sQuaternion* temp = new sQuaternion[target.Keys];
    sF32 sign = 1.0f;

    for(sInt i=0;i<target.Keys;i++)
    {
      if(i && dot(target.Rot[i-1],target.Rot[i]) < 0.0f)
        sign = -sign;

      temp[i] = sign * target.Rot[i];
    }

    sBool isOk = Rot.FitToCurve(temp,target.Keys,err,1.0f);
    ok &= isOk;
    delete[] temp;
  }

  if(Scale.IsConstant(err)) { Scale.Evaluate((sVector30&) Start.Scale,0.0f); Scale.Clear(); }
  if(Rot.IsConstant(err))   { Rot.Evaluate(Start.Rot,0.0f); Rot.Clear(); }
  if(Trans.IsConstant(err)) { Trans.Evaluate((sVector30&) Start.Trans,0.0f); Trans.Clear(); }

  return ok;
}

void Wz4ChannelSpline::Evaluate(sF32 time,Wz4AnimKey &result)
{
  result = Start;
  time /= MaxTime;

  if(!Scale.IsEmpty())  Scale.Evaluate((sVector30&) result.Scale,time);
  if(!Rot.IsEmpty())    Rot.Evaluate(result.Rot,time);
  if(!Trans.IsEmpty())  Trans.Evaluate((sVector30&) result.Trans,time);
}

/****************************************************************************/

Wz4ChannelCat::Wz4ChannelCat()
{
  Kind = Wz4ChannelKindCat;
}

Wz4ChannelCat::~Wz4ChannelCat()
{
  sReleaseAll(Channels);
}

void Wz4ChannelCat::Evaluate(sF32 time,Wz4AnimKey &result)
{
  Wz4AnimKey a,b;
  Wz4Channel *ch;
  sFORALL(Channels,ch)
  {
    if(_i==0)
    {
      ch->Evaluate(time,result);
    }
    else
    {
      ch->Evaluate(time,a);
      result.Trans = result.Trans + sVector30(a.Trans);
      result.Scale.x = result.Scale.x * a.Scale.x;
      result.Scale.y = result.Scale.y * a.Scale.y;
      result.Scale.z = result.Scale.z * a.Scale.z;
      result.Rot = result.Rot * a.Rot;
    }
  }
}

/****************************************************************************/
/***                                                                      ***/
/***   Skeleton                                                           ***/
/***                                                                      ***/
/****************************************************************************/

Wz4Skeleton::Wz4Skeleton()
{
  Type = Wz4SkeletonType;
  TotalTime = 1;
}

Wz4Skeleton::~Wz4Skeleton()
{
  Wz4AnimJoint *j;
  sFORALL(Joints,j)
    j->Channel->Release();
}

template <class streamer> void Wz4Skeleton::Serialize_(streamer &stream)
{
  Wz4AnimJoint *joint;
  sInt version = stream.Header(sSerId::Wz4Skeleton,2);
  if(version)
  {
    stream.Array(Joints);
    sFORALL(Joints,joint)
    {
      stream | joint->Name | joint->Parent | joint->BasePose;
      SerializeWz4Channel(stream,joint->Channel);
    }
    if(version>=2)
    {
      stream | TotalTime;
    }
    else
    {
      FixTime(1/60.0f);  // assuming 60 fps
    }
    stream.Footer();
  }
}
void Wz4Skeleton::Serialize(sWriter &stream) { Serialize_(stream); }
void Wz4Skeleton::Serialize(sReader &stream) { Serialize_(stream); }


void Wz4Skeleton::CopyFrom(Wz4Skeleton *src)
{
  Wz4AnimJoint *joint;
  Joints = src->Joints;
  TotalTime = src->TotalTime;
  sFORALL(Joints,joint)
    if(joint->Channel)
      joint->Channel = joint->Channel->CopyTo();
}

sInt Wz4Skeleton::FindJoint(const sChar *name)
{
  Wz4AnimJoint *joint;
  sFORALL(Joints,joint)
    if(sCmpString(name,joint->Name)==0)
      return _i;
  return -1;
}

void Wz4Skeleton::FixTime(sF32 spf)
{
  sInt max = 0;
  Wz4AnimJoint *joint;
  sFORALL(Joints,joint)
    if(joint->Channel->Kind==Wz4ChannelKindPerFrame)
      max = sMax(max,((Wz4ChannelPerFrame *)(joint->Channel))->Keys);
  TotalTime = max*spf;
}


void Wz4Skeleton::Evaluate(sF32 time,sMatrix34 *mata,sMatrix34 *basemat)
{
  Wz4AnimJoint *j;
  Wz4AnimKey key;
  sMatrix34 mat;

  if(0) // softimage hirachical scaling
  {
    sVector31 *scale = sALLOCSTACK(sVector31,Joints.GetCount());
    sVector31 trans;
    sMatrix34 *rotat = sALLOCSTACK(sMatrix34,Joints.GetCount());
    sMatrix34 *local = sALLOCSTACK(sMatrix34,Joints.GetCount());
    sFORALL(Joints,j)
    {
      sInt i = _i;
      if(j->Channel)
      {
        j->Channel->Evaluate(time,key);
      }
      else
      {
        key.Init();
      }
      if(j->Parent==-1)
      {
        scale[i] = key.Scale;
        trans = key.Trans;
        rotat[i].Init(key.Rot);
        key.ToMatrix(mat);
        local[i] = mat;
      }
      else
      {
        mat.Init(key.Rot);
        scale[i] = key.Scale * scale[j->Parent];
        rotat[i] = mat * rotat[j->Parent];
        trans = key.Trans * local[j->Parent];
        key.ToMatrix(mat);
        local[i] = mat * local[j->Parent];
      }

      sMatrix34 mat1,mat2,mat3;

      mat = rotat[i];
      mat.Scale(scale[i].x,scale[i].y,scale[i].z);
      mat.l = trans;
      mata[i] = mat;

      if(basemat)
        basemat[i] = j->BasePose * mata[i];
    }     
  }
  else
  {
    sFORALL(Joints,j)
    {
      if(j->Channel)
      {
        j->Channel->Evaluate(time,key);
        key.ToMatrix(mat);
      }
      else
      {
        mat.Init();
      }
      if(j->Parent==-1)
        mata[_i] = mat;
      else
        mata[_i] = mat * mata[j->Parent];
      if(basemat)
        basemat[_i] = j->BasePose * mata[_i];
    }
  }
}

/****************************************************************************/
// ASSIMP TEST
/****************************************************************************/

void VertexBoneData::AddBoneData(sU32 BoneID, float Weight)
{
    for (sU32 i = 0 ; i < ARRAY_SIZE_IN_ELEMENTS(IDs) ; i++)
    {
        if (Weights[i] == 0.0)
        {
            IDs[i] = BoneID;
            Weights[i] = Weight;
            return;
        }
    }

    // should never get here - more bones than we have space for
    assert(0);
}

void Matrix4f(sMatrix34 & wzmat, const aiMatrix4x4 & aimat)
{  
  wzmat.i.x =  aimat.a1;
  wzmat.i.y =  aimat.a2;
  wzmat.i.z =  aimat.a3;

  wzmat.j.x =  aimat.b1;
  wzmat.j.y =  aimat.b2;
  wzmat.j.z =  aimat.b3;

  wzmat.k.x =  aimat.c1;
  wzmat.k.y =  aimat.c2;
  wzmat.k.z =  aimat.c3;

  wzmat.l.x =  aimat.d1;
  wzmat.l.y =  aimat.d2;
  wzmat.l.z =  aimat.d3;
}

const aiNodeAnim* Wz4Skeleton::FindNodeAnim(const aiAnimation* pAnimation, const std::string NodeName)
{
    for (sU32 i = 0 ; i < pAnimation->mNumChannels ; i++) {
        const aiNodeAnim* pNodeAnim = pAnimation->mChannels[i];
        
        if (std::string(pNodeAnim->mNodeName.data) == NodeName) {
            return pNodeAnim;
        }
    }
    
    return NULL;
}

sU32 Wz4Skeleton::FindPosition(float AnimationTime, const aiNodeAnim* pNodeAnim)
{    
    for (sU32 i = 0 ; i < pNodeAnim->mNumPositionKeys - 1 ; i++) {
        if (AnimationTime < (float)pNodeAnim->mPositionKeys[i + 1].mTime) {
            return i;
        }
    }
    
    assert(0);

    return 0;
}

sU32 Wz4Skeleton::FindRotation(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    assert(pNodeAnim->mNumRotationKeys > 0);

    for (sU32 i = 0 ; i < pNodeAnim->mNumRotationKeys - 1 ; i++) {
        if (AnimationTime < (float)pNodeAnim->mRotationKeys[i + 1].mTime) {
            return i;
        }
    }
    
    assert(0);

    return 0;
}

sU32 Wz4Skeleton::FindScaling(float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    assert(pNodeAnim->mNumScalingKeys > 0);
    
    for (sU32 i = 0 ; i < pNodeAnim->mNumScalingKeys - 1 ; i++) {
        if (AnimationTime < (float)pNodeAnim->mScalingKeys[i + 1].mTime) {
            return i;
        }
    }
    
    assert(0);

    return 0;
}

void Wz4Skeleton::CalcInterpolatedScaling(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    if (pNodeAnim->mNumScalingKeys == 1) {
        Out = pNodeAnim->mScalingKeys[0].mValue;
        return;
    }

    sU32 ScalingIndex = FindScaling(AnimationTime, pNodeAnim);
    sU32 NextScalingIndex = (ScalingIndex + 1);
    assert(NextScalingIndex < pNodeAnim->mNumScalingKeys);
    float DeltaTime = (float)(pNodeAnim->mScalingKeys[NextScalingIndex].mTime - pNodeAnim->mScalingKeys[ScalingIndex].mTime);
    float Factor = (AnimationTime - (float)pNodeAnim->mScalingKeys[ScalingIndex].mTime) / DeltaTime;
    assert(Factor >= 0.0f && Factor <= 1.0f);
    const aiVector3D& Start = pNodeAnim->mScalingKeys[ScalingIndex].mValue;
    const aiVector3D& End   = pNodeAnim->mScalingKeys[NextScalingIndex].mValue;
    aiVector3D Delta = End - Start;
    Out = Start + Factor * Delta;
}

void Wz4Skeleton::CalcInterpolatedRotation(aiQuaternion& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    // we need at least two values to interpolate...
    if (pNodeAnim->mNumRotationKeys == 1) {
        Out = pNodeAnim->mRotationKeys[0].mValue;
        return;
    }

    sU32 RotationIndex = FindRotation(AnimationTime, pNodeAnim);
    sU32 NextRotationIndex = (RotationIndex + 1);
    assert(NextRotationIndex < pNodeAnim->mNumRotationKeys);
    float DeltaTime = pNodeAnim->mRotationKeys[NextRotationIndex].mTime - pNodeAnim->mRotationKeys[RotationIndex].mTime;
    float Factor = (AnimationTime - (float)pNodeAnim->mRotationKeys[RotationIndex].mTime) / DeltaTime;
    assert(Factor >= 0.0f && Factor <= 1.0f);
    const aiQuaternion& StartRotationQ = pNodeAnim->mRotationKeys[RotationIndex].mValue;
    const aiQuaternion& EndRotationQ = pNodeAnim->mRotationKeys[NextRotationIndex].mValue;
    aiQuaternion::Interpolate(Out, StartRotationQ, EndRotationQ, Factor);
    Out = Out.Normalize();
} 

void Wz4Skeleton::CalcInterpolatedPosition(aiVector3D& Out, float AnimationTime, const aiNodeAnim* pNodeAnim)
{
    if (pNodeAnim->mNumPositionKeys == 1) {
        Out = pNodeAnim->mPositionKeys[0].mValue;
        return;
    }
            
    sU32 PositionIndex = FindPosition(AnimationTime, pNodeAnim);
    sU32 NextPositionIndex = (PositionIndex + 1);
    assert(NextPositionIndex < pNodeAnim->mNumPositionKeys);
    float DeltaTime = (float)(pNodeAnim->mPositionKeys[NextPositionIndex].mTime - pNodeAnim->mPositionKeys[PositionIndex].mTime);
    float Factor = (AnimationTime - (float)pNodeAnim->mPositionKeys[PositionIndex].mTime) / DeltaTime;
    assert(Factor >= 0.0f && Factor <= 1.0f);
    const aiVector3D& Start = pNodeAnim->mPositionKeys[PositionIndex].mValue;
    const aiVector3D& End = pNodeAnim->mPositionKeys[NextPositionIndex].mValue;
    aiVector3D Delta = End - Start;
    Out = Start + Factor * Delta;
}

const aiScene * scene;
Assimp::Importer m_importer;


//void Wz4Skeleton::ReadNodeHeirarchy(sF32 AnimationTime, const aiNode* pNode , const sMatrix34 & ParentTransform)
void Wz4Skeleton::ReadNodeHeirarchy(sF32 AnimationTime, const aiNode* pNode , const aiMatrix4x4 & ParentTransform)
{

  std::string NodeName(pNode->mName.data);
  const aiAnimation* pAnimation = scene->mAnimations[0];

  //sMatrix34 NodeTransformation;
  //Matrix4f(NodeTransformation, pNode->mTransformation);
  aiMatrix4x4 NodeTransformation = pNode->mTransformation;


  const aiNodeAnim* pNodeAnim = FindNodeAnim(pAnimation, NodeName);

   if (pNodeAnim) {

         sSRT srt;

        // Interpolate scaling and generate scaling transformation matrix
        aiVector3D Scaling;
        CalcInterpolatedScaling(Scaling, AnimationTime, pNodeAnim);
        //Matrix4f ScalingM;
        //ScalingM.InitScaleTransform(Scaling.x, Scaling.y, Scaling.z);
        
        /*sMatrix34 ScalingM;        
        srt.Scale.x = Scaling.x;
        srt.Scale.y = Scaling.y;
        srt.Scale.z = Scaling.z;
        srt.Rotate = sVector30(0);
        srt.Translate = sVector31(0);
        srt.MakeMatrix(ScalingM);*/

        aiMatrix4x4 ScalingM(Scaling, aiQuaternion(), aiVector3t<sF32>(0));
        

        // Interpolate rotation and generate rotation transformation matrix
        aiQuaternion RotationQ;
        CalcInterpolatedRotation(RotationQ, AnimationTime, pNodeAnim);
        //Matrix4f RotationM = Matrix4f(RotationQ.GetMatrix());
        
        /*sMatrix34 RotationM;
        sQuaternion quat;
        quat.Init(RotationQ.w, RotationQ.x, RotationQ.y, RotationQ.z);
        RotationM.Init(quat);*/

        aiMatrix4x4 RotationM(aiVector3t<sF32>(1), RotationQ, aiVector3t<sF32>(0));



        // Interpolate translation and generate translation transformation matrix
        aiVector3D Translation;
        CalcInterpolatedPosition(Translation, AnimationTime, pNodeAnim);
        //Matrix4f TranslationM;
        //TranslationM.InitTranslationTransform(Translation.x, Translation.y, Translation.z);
        
        /*sMatrix34 TranslationM;
        srt.Scale = sVector31(1);
        srt.Rotate = sVector30(0);
        srt.Translate.x = Translation.x;
        srt.Translate.y = Translation.y;
        srt.Translate.z = Translation.z;
        srt.MakeMatrix(TranslationM);*/

        aiMatrix4x4 TranslationM(aiVector3t<sF32>(1), aiQuaternion(), Translation);


        // Combine the above transformations
        NodeTransformation = TranslationM * RotationM * ScalingM;       



        /*srt.Scale.x = Scaling.x;
        srt.Scale.y = Scaling.y;
        srt.Scale.z = Scaling.z;

        srt.Rotate = sVector30(0);

        srt.Translate.x = Translation.x;
        srt.Translate.y = Translation.y;
        srt.Translate.z = Translation.z;

        srt.MakeMatrix(NodeTransformation);

        sVector30 axis;
        sF32 angle;
        quat.GetAxisAngle(axis, angle);

        NodeTransformation.RotateAxis(axis,angle);*/
        //NodeTransformation.Init(quat);


    }

   // Matrix4f GlobalTransformation = ParentTransform * NodeTransformation;
   //sMatrix34 GlobalTransformation;
   //GlobalTransformation = ParentTransform * NodeTransformation;
   aiMatrix4x4 GlobalTransformation = ParentTransform * NodeTransformation;


    if (m_BoneMapping.find(NodeName) != m_BoneMapping.end()) 
    {
        sU32 BoneIndex = m_BoneMapping[NodeName];
        m_BoneInfo[BoneIndex].FinalTransformation = m_GlobalInverseTransform * GlobalTransformation * m_BoneInfo[BoneIndex].BoneOffset;
    }

    for (sU32 i = 0 ; i < pNode->mNumChildren ; i++) 
    {
        ReadNodeHeirarchy(AnimationTime, pNode->mChildren[i], GlobalTransformation);
    }








  /*wDocName name;
  sCopyString(name, pNode->mName.C_Str(), pNode->mName.length);

  sMatrix34 NodeTransformation;  
  Matrix4f(NodeTransformation, pNode->mTransformation);


  sInt index = sFindIndex(Joints, &Wz4AnimJoint::Name, name);
  if(index != -1)
  {
    Wz4ChannelPerFrame * c = static_cast<Wz4ChannelPerFrame *>(Joints[index].Channel);
    
    sMatrix34 mat;
    Wz4AnimKey key;
    c->Evaluate(time, key);
    key.ToMatrix(mat);   
  }

  sMatrix34 GlobalTransformation;
  GlobalTransformation = ParentTransform * NodeTransformation;

  if(index != -1)
  {
    Joints[index].aiFinalTransformation = GlobalTransformation * Joints[index].aiBoneOffset;
  }


  for(sU32 i=0; i<pNode->mNumChildren; i++) 
  {
    ReadNodeHeirarchy(time, pNode->mChildren[i], GlobalTransformation);
  }
  */

}

void Wz4Skeleton::EvaluateCM(sF32 time,sMatrix34 *mata,sMatrix34CM *basemat)
{
  Wz4AnimJoint *j;
  Wz4AnimKey key;
  sMatrix34 mat;
  

  float TicksPerSecond = scene->mAnimations[0]->mTicksPerSecond != 0 ? scene->mAnimations[0]->mTicksPerSecond : 25.0f;
  float TimeInTicks = time * TicksPerSecond;
  float AnimationTime = fmod(TimeInTicks, scene->mAnimations[0]->mDuration);
  
  //sMatrix34 identity;
  //identity.Init();

  aiMatrix4x4 identity;


  ReadNodeHeirarchy(AnimationTime, scene->mRootNode, identity);

   for (sU32 i=0; i<m_NumBones; i++) 
   {
     //Matrix4f(sMatrix34(basemat[i]), m_BoneInfo[i].FinalTransformation);

     //m_BoneInfo[i].FinalTransformation.Trans3();
     //basemat[i] = m_BoneInfo[i].FinalTransformation;

     basemat[i].Init();

     basemat[i].x.x = m_BoneInfo[i].FinalTransformation.a1;
     basemat[i].x.y = m_BoneInfo[i].FinalTransformation.a2;
     basemat[i].x.z = m_BoneInfo[i].FinalTransformation.a3;
     basemat[i].x.w = m_BoneInfo[i].FinalTransformation.a4;

     basemat[i].y.x = m_BoneInfo[i].FinalTransformation.b1;
     basemat[i].y.y = m_BoneInfo[i].FinalTransformation.b2;
     basemat[i].y.z = m_BoneInfo[i].FinalTransformation.b3;
     basemat[i].y.w = m_BoneInfo[i].FinalTransformation.b4;

     basemat[i].z.x = m_BoneInfo[i].FinalTransformation.c1;
     basemat[i].z.y = m_BoneInfo[i].FinalTransformation.c2;
     basemat[i].z.z = m_BoneInfo[i].FinalTransformation.c3;
     basemat[i].z.w = m_BoneInfo[i].FinalTransformation.c4;

    
   }


  /*sFORALL(Joints,j)
  {
    basemat[_i] = j->aiFinalTransformation;
  }*/






  

  /*sFORALL(Joints,j)
  {
    if(j->Channel)
    {
      j->Channel->Evaluate(time,key);
      key.ToMatrix(mat);
    }
    else
    {
      mat.Init();
    }
    if(j->Parent==-1)
      mata[_i] = mat;
    else
    {

      //mata[_i] = mat *mata[j->Parent]; // bad

      sMatrix34 keyMatParent;
      Joints[j->Parent].Channel->Evaluate(time,key);
      key.ToMatrix(keyMatParent);

      //mata[_i] = mat * matParent;
      mata[_i] = mat * Joints[j->Parent].BasePose * keyMatParent;

    }
    basemat[_i] = j->BasePose * mata[_i];
  }*/


  
 /*j->Channel

  sFORALL(Joints, j)
  {
    if(j->Parent == -1)
    {
      mat.Init();
      j->BasePose = j->BasePose;
    }
    else
    {
      j->BasePose *=  Joints[j->Parent].BasePose;
    }

    basemat[_i] = j->BasePose;
  }*/

  
 

  /*sFORALL(Joints,j)
  {
    if(j->Channel)
    {
      j->Channel->Evaluate(time,key);
      key.ToMatrix(mat);
    }
    else
    {
      mat.Init();
    }
  }*/
}

/****************************************************************************/
/****************************************************************************/

void Wz4Skeleton::EvaluateBlendCM(sF32 time1,sF32 time2,sF32 reftime,sMatrix34 *mata,sMatrix34CM *basemat)
{
  Wz4AnimJoint *j;
  Wz4AnimKey key;
  sMatrix34 mat,mat1,mat2,mat2r;

  sFORALL(Joints,j)
  {
    if(j->Channel)
    {
      j->Channel->Evaluate(time1,key);
      key.ToMatrix(mat1);
      j->Channel->Evaluate(time2,key);
      key.ToMatrix(mat2);
      j->Channel->Evaluate(reftime,key);
      key.ToMatrixInv(mat2r);
      mat = mat1*mat2r;
      mat = mat*mat2;
    }
    else
    {
      mat.Init();
    }
    if(j->Parent==-1)
      mata[_i] = mat;
    else
      mata[_i] = mat * mata[j->Parent];
    basemat[_i] = j->BasePose * mata[_i];
  }
}

void Wz4Skeleton::EvaluateFadeCM(sF32 time1,sF32 time2,sF32 fade,sMatrix34 *mata,sMatrix34CM *basemat)
{
  Wz4AnimJoint *j;
  Wz4AnimKey key;
  sMatrix34 mat,mat1,mat2;

  sFORALL(Joints,j)
  {
    if(j->Channel)
    {
      if(_i>=10 && _i<161)      // detuned fake: explodierende augäpfel
      {
        j->Channel->Evaluate(time1,key);
        key.ToMatrix(mat);
      }
      else
      {
        j->Channel->Evaluate(time1,key);
        key.ToMatrix(mat1);
        j->Channel->Evaluate(time2,key);
        key.ToMatrix(mat2);
        mat.i.Fade(fade,mat1.i,mat2.i); mat.i.Unit();
        mat.j.Fade(fade,mat1.j,mat2.j); mat.j.Unit();
        mat.k.Fade(fade,mat1.k,mat2.k); mat.k.Unit();
  //      mat.k.Cross(mat.i,mat.j); mat.k.Unit();
  //      mat.i.Cross(mat.j,mat.k); mat.i.Unit();
        mat.l.Fade(fade,mat1.l,mat2.l);
      }
    }
    else
    {
      mat.Init();
    }
    if(j->Parent==-1)
      mata[_i] = mat;
    else
      mata[_i] = mat * mata[j->Parent];
    basemat[_i] = j->BasePose * mata[_i];
  }
}

/****************************************************************************/
