using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;




namespace OctopusController
{


    internal class MyTentacleController

    //MAINTAIN THIS CLASS AS INTERNAL
    {

        TentacleMode tentacleMode;
        Transform[] _bones;
        Transform _endEffectorSphere;
        Transform _base;
        Vector3[] axis;
        Vector3[] startOffset;

        float[] theta;
        Vector3 targetPositionCCD;
        int currentTries;
        float[] distances;

        public Transform[] Bones { get => _bones; }
        public Transform EndEffector { get => _endEffectorSphere; }
        public Vector3 TargetPositionCCD { get { return targetPositionCCD; } set { targetPositionCCD = value; } }
        public int CurrentTries { get => currentTries; set { currentTries = value; } }
        public float[] Theta { get => theta; }


        //Exercise 1.
        public Transform[] LoadTentacleJoints(Transform root, TentacleMode mode)
        {
            //TODO: add here whatever is needed to find the bones forming the tentacle for all modes
            List<Transform> bones = new List<Transform>();
            Transform current = root;

            //you may want to use a list, and then convert it to an array and save it into _bones
            tentacleMode = mode;

            switch (tentacleMode)
            {
                case TentacleMode.LEG:
                    current = current.GetChild(0).transform;
                    bones.Add(current);
                    while (current.childCount > 0)
                    {
                        current = current.GetChild(1).transform;
                        bones.Add(current);
                    }
                    _endEffectorSphere = current;
                    bones.Remove(current);
                    _bones = bones.ToArray();

                    _base = _bones[0];
                    theta = new float[_bones.Length];
                    for (int i = 0; i < _bones.Length; i++)
                        theta[i] = 0;

                    startOffset = new Vector3[_bones.Length];
                    for (int i = 1; i < startOffset.Length; i++)
                        startOffset[i] = _bones[i].position - _bones[i - 1].position;

                    distances = new float[_bones.Length - 1];
                    for (int i = 0; i < _bones.Length - 1; i++)
                        distances[i] = Vector3.Distance(_bones[i].position, _bones[i + 1].position);
                    //TODO: in _endEffectorsphere you keep a reference to the base of the leg
                    break;
                case TentacleMode.TAIL:
                    bones.Add(current);
                    while (current.childCount > 0)
                    {
                        current = current.GetChild(1).transform;
                        bones.Add(current);
                    }
                    _endEffectorSphere = current.parent.GetChild(0).transform;
                    bones.Remove(current);
                    _bones = bones.ToArray();

                    _base = _bones[0];
                    theta = new float[_bones.Length];
                    for (int i = 0; i < _bones.Length; i++)
                        theta[i] = 0;

                    startOffset = new Vector3[_bones.Length];
                    for (int i = 1; i < startOffset.Length; i++)
                        startOffset[i] = _bones[i].position - _bones[i - 1].position;

                    distances = new float[_bones.Length - 1];
                    for (int i = 0; i < _bones.Length - 1; i++)
                        distances[i] = Vector3.Distance(_bones[i].position, _bones[i + 1].position);
                    //TODO: in _endEffectorsphere you keep a reference to the red sphere 
                    break;
                case TentacleMode.TENTACLE:
                    current = current.GetChild(0).transform;
                    while (current.childCount > 0)
                    {
                        current = current.GetChild(0).transform;
                        bones.Add(current);
                    }
                    _endEffectorSphere = current;
                    bones.Remove(current);
                    _bones = bones.ToArray();

                    _base = _bones[0];
                    theta = new float[_bones.Length];
                    for (int i = 0; i < _bones.Length; i++)
                        theta[i] = 0;

                    startOffset = new Vector3[_bones.Length];
                    for (int i = 1; i < startOffset.Length; i++)
                        startOffset[i] = _bones[i].position - _bones[i - 1].position;

                    distances = new float[_bones.Length - 1];
                    for (int i = 0; i < _bones.Length - 1; i++)
                        distances[i] = Vector3.Distance(_bones[i].position, _bones[i + 1].position);
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    break;
            }
            Debug.Log(root.gameObject.name + ": " + _bones.Length + "bones" + ", EndEffector: " + _endEffectorSphere.gameObject.name);
            return Bones;
        }

        void SetAxis()
        {
            axis = new Vector3[_bones.Length];

            for (int i = 1; i < _bones.Length; i++)
            {
                int axisID = i % 2;
                switch (axisID)
                {
                    case 0:
                        axis[i] = Vector3.right;
                        break;
                    case 1:
                        axis[i] = Vector3.forward;
                        break;
                }
            }
        }
    }
}
