////////////////////////////////////////////////////////////////////////////////
// blendshape.cpp
// Author     : Francesco Giordana
// Start Date : January 13, 2005
// Copyright  : (C) 2006 by Francesco Giordana
// Email      : fra.giordana@tiscali.it
////////////////////////////////////////////////////////////////////////////////

/*********************************************************************************
*                                                                                *
*   This program is free software; you can redistribute it and/or modify         *
*   it under the terms of the GNU Lesser General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or            *
*   (at your option) any later version.                                          *
*                                                                                *
**********************************************************************************/

#include "blendshape.h"
#include "submesh.h"
#include "FxOgreFBXLog.h"
namespace FxOgreFBX
{
    // Constructor
    BlendShape::BlendShape(FbxNode* pNode, const FbxAMatrix& mat)
    {
        clear();
        m_pNode = pNode;
        m_bindPose = mat;
    }

    // Destructor
    BlendShape::~BlendShape()
    {
        clear();
    }

    // Clear blend shape data
    void BlendShape::clear()
    {
        m_pNode = NULL;
        m_origWeights.clear();
        m_poseGroups.clear();
        poseGroup pg;
        pg.targetIndex = 0;
        m_poseGroups.insert(std::pair<int,poseGroup>(0, pg));
        m_target = T_MESH;
    }

    void BlendShape::getKeyedFrames(std::vector<int>& keyedFrames )
    {
        for( size_t i = 0; i < m_poseGroups.size(); ++i )
        {
            for( size_t j = 0; j <  m_poseGroups[i].poses.size(); ++j )
            {
                AppendKeyedFrames(keyedFrames, m_poseGroups[i].poses[j].pCurve);
            }
        }
    }
    // Load blend shape poses
    bool BlendShape::loadPoses(ParamList &params, std::vector<vertex> &vertices,long numVertices,long offset,long targetIndex)
    {
        if (params.useSharedGeom)
        {
            assert(targetIndex == 0);
            m_target = T_MESH;
        }
        else
        {
            assert(offset == 0);
            poseGroup new_pg;
            m_target = T_SUBMESH;
            new_pg.targetIndex = targetIndex;
            m_poseGroups.insert(std::pair<int,poseGroup>(targetIndex,new_pg));
        }
        poseGroup& pg = m_poseGroups.find(targetIndex)->second;

        // Get the animstack for getting blendshape curve info.
        FbxAnimLayer* pAnimLayer = NULL;
        FbxAnimStack* pAnimStack = FbxCast<FbxAnimStack>(params.pScene->GetSrcObject(FBX_TYPE(FbxAnimStack), 0));
        if( pAnimStack)
        {
            pAnimLayer = pAnimStack->GetMember(FBX_TYPE(FbxAnimLayer), 0);
        }

        FxOgreFBXLog( "Exporting Blendshapes with bindmatrix:\n");
        logMatrix(m_bindPose);

        FbxMesh *pMesh = m_pNode->GetMesh();
        if( pMesh )
        {
            int ogreBlendShapeIndex = 0;
            int lBlendShapeDeformerCount = pMesh->GetDeformerCount(FbxDeformer::eBlendShape);

            for(int lBlendShapeIndex = 0; lBlendShapeIndex<lBlendShapeDeformerCount; ++lBlendShapeIndex)
            {
                FbxBlendShape* lBlendShape = (FbxBlendShape*)pMesh->GetDeformer(lBlendShapeIndex, FbxDeformer::eBlendShape);

                FxOgreFBXLog( "Exporting BlendshapeDeformer %s.\n", lBlendShape->GetName());
                int lBlendShapeChannelCount = lBlendShape->GetBlendShapeChannelCount();
                for(int lChannelIndex = 0; lChannelIndex<lBlendShapeChannelCount; ++lChannelIndex)
                {
                    FbxBlendShapeChannel* lChannel = lBlendShape->GetBlendShapeChannel(lChannelIndex);
                    if(lChannel)
                    {
                        FxOgreFBXLog( "Exporting BlendShapeChannel %s.\n", lChannel->GetName());

                        // Looping through lChannel->GetTargetShapeCount() only gives me duplicate shapes.
                        // Just see if index 0 exists.
                        FbxShape* pShape = lChannel->GetTargetShape(0);
                        if(pShape)
                        {				
                            std::string posename = pShape->GetName();
                            int numMorphVertices = pShape->GetControlPointsCount();
                            int numIndices = pShape->GetControlPointIndicesCount();

                            FxOgreFBXLog( "Exporting Morph target: %s with %d vertices.\n", posename.c_str(), numIndices);
                            FxOgreFBXLog( "Mesh has %d vertices.\n", numVertices);
                            FxOgreFBXLog( "%d total vertices.\n", vertices.size());

                            assert(offset+numVertices <= (long)vertices.size());

                            // create a new pose
                            pose p;
                            p.poseTarget = m_target;
                            p.index = targetIndex;
                            p.blendShapeIndex = ogreBlendShapeIndex;
                            p.name = posename;
                            p.pShape = pShape;

                            p.pCurve = pMesh->GetShapeChannel(lBlendShapeIndex, lChannelIndex, pAnimLayer);

                            ogreBlendShapeIndex++;
                            
                            
                            size_t numPoints =  pMesh->GetControlPointsCount();
                            std::vector<FbxVector4> vmPoints;
                            vmPoints.reserve(numPoints);

                            // Get original verts.
                            for( size_t k = 0; k < numPoints; ++k )
                            {
                                vmPoints.push_back( pMesh->GetControlPointAt(k) );
                            }
                            
                            // Replace with deformed verts.
                            for( int k = 0; k < numIndices; ++k )
                            {
                                int index = pShape->GetControlPointIndices()[k];
                                assert(index < (int)vmPoints.size() && index >= 0);
                                vmPoints[index] =  pShape->GetControlPointAt(index);
                            }
                        
                            
                            BoundingBox morphBoundingBox;
                            
                            // calculate vertex offsets
                            for (int k=0; k<numVertices; k++)
                            {
                                vertexOffset vo;
                                assert ((offset+k)< (int)vertices.size());

                                vertex v = vertices[offset+k];
                                assert(v.index < numMorphVertices);
                                assert(v.index < pMesh->GetControlPointsCount());

                                FbxVector4 meshVert = m_bindPose.MultT( pMesh->GetControlPointAt(v.index) );

                                int morphindex = pShape->GetControlPointIndices()[v.index];
                                FbxVector4 morphVert = m_bindPose.MultT( vmPoints[v.index] );

                                FbxVector4 diff = morphVert - meshVert;

                                // Add this point to the bounding box
                                morphBoundingBox.merge(Point3(morphVert[0], morphVert[1], morphVert[2]));

                                vo.x = static_cast<float>(diff[0] * params.lum);
                                vo.y = static_cast<float>(diff[1] * params.lum);
                                vo.z = static_cast<float>(diff[2] * params.lum);	

                                vo.index = offset+k;
                                if (fabs(vo.x) < PRECISION)
                                    vo.x = 0;
                                if (fabs(vo.y) < PRECISION)
                                    vo.y = 0;
                                if (fabs(vo.z) < PRECISION)
                                    vo.z = 0;
                                if ((vo.x!=0) || (vo.y!=0) || (vo.z!=0))
                                    p.offsets.push_back(vo);
                            }

                            // Unfortunately we can't prune zero vertex shapes without
                            // requiring full geometry animation files.  (Calling this 
                            // function with numVertices=0 should still set up pg.poses).
                            pg.poses.push_back(p);

                            if (params.bsBB)
                            {
                                // update bounding boxes of loaded submeshes
                                for (size_t j=0; j<params.loadedSubmeshes.size(); j++)
                                {
                                    Point3 min = Point3(morphBoundingBox.min.x * params.lum, morphBoundingBox.min.y * params.lum, morphBoundingBox.min.z * params.lum);
                                    Point3 max = Point3(morphBoundingBox.max.x * params.lum, morphBoundingBox.max.y * params.lum, morphBoundingBox.max.z * params.lum);

                                    params.loadedSubmeshes[j]->m_bbox.merge(min);
                                    params.loadedSubmeshes[j]->m_bbox.merge(max);
                                }
                            }
                        }
                    }
                }
            }        
        }
        return true;
    }

    // Load a blend shape animation track
    Track BlendShape::loadTrack(float start,float stop,float rate,ParamList& params,int targetIndex, int startPoseId)
    {
        std::string msg;
        std::vector<float> times;
        // Create a track for current clip
        Track t;
        t.m_type = TT_POSE;
        t.m_target = m_target;
        t.m_index = targetIndex;
        t.m_vertexKeyframes.clear();
        // Calculate times from clip sample rate
        times.clear();
        if (rate <= 0)
        {
            FxOgreFBXLog( "invalid sample rate for the clip (must be >0), we skip it\n");
            return t;
        }
        float time;
        for (time=start; time<stop; time+=rate)
            times.push_back(time);
        times.push_back(stop);
        // Get animation length
        float length=0;
        if (times.size() >= 0)
            length = times[times.size()-1] - times[0];
        if (length < 0)
        {
            FxOgreFBXLog( "invalid time range for the clip, we skip it\n");
            return t;
        }
        poseGroup& pg = m_poseGroups.find(targetIndex)->second;
        FbxMesh *pMesh = m_pNode->GetMesh();
        if( pMesh )
        {
            // Evaluate animation curves at selected times
            for (size_t i=0; i<times.size(); i++)
            {
                float time = times[i] - times[0];
                vertexKeyframe key = loadKeyframe(time, pg, startPoseId);
                key.time = key.time - times[0];
                t.addVertexKeyframe(key);
            }
        }
        // Clip successfully loaded
        return t;
    }

    // Load a blend shape animation keyframe
    vertexKeyframe BlendShape::loadKeyframe(float time, poseGroup& pg, int startPoseId )
    {
        vertexKeyframe key;
        key.time = time;
        key.poserefs.clear();
        FbxMesh *pMesh = m_pNode->GetMesh();
        if( pMesh )
        {        
            for (size_t j=0; j<pg.poses.size(); j++)
            {
                pose& p = pg.poses[j];
                if( p.pCurve )
                {
                    FbxTime fbxtime;
                    fbxtime.SetSecondDouble(key.time);
                    float value = p.pCurve->Evaluate(fbxtime);

                    vertexPoseRef poseref;
                    poseref.poseIndex = startPoseId + j;
                    poseref.poseWeight = value/100;
                    key.poserefs.push_back(poseref);
                }
            }
        }

        return key;
    }
    

    // Get blend shape poses
    stdext::hash_map<int,poseGroup>& BlendShape::getPoseGroups()
    {
        return m_poseGroups;
    }

} // end namespace
