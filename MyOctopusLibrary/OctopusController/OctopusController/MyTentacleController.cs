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
        Transform[] _joints;
        Transform _endEffectorSphere;
        Transform _base;
        Vector3[] axis;
        Vector3[] startOffset;
        Vector3[] initRotation;

        float[] theta;
        Vector3 targetPositionCCD;
        int currentTries;

        public Transform Base { get => _base; }
        public Transform[] Joints { get => _joints; }
        public Transform EndEffector { get => _endEffectorSphere; }
        public Vector3 TargetPositionCCD { get { return targetPositionCCD; } set { targetPositionCCD = value; } }
        public int CurrentTries { get => currentTries; set { currentTries = value; } }
        public Vector3[] Axis { get => axis; }
        public Vector3[] StartOffset { get => startOffset; }
        public Vector3[] InitRotation { get => initRotation; }
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
                    _joints = bones.ToArray();

                    _base = _joints[0];
                    theta = new float[_joints.Length];
                    for (int i = 0; i < _joints.Length; i++)
                        theta[i] = 0;
                    //TODO: in _endEffectorsphere you keep a reference to the base of the leg
                    break;
                case TentacleMode.TAIL:
                    bones.Add(current);
                    while (current.childCount > 0)
                    {
                        current = current.GetChild(1).transform;
                        bones.Add(current);
                    }
                    _endEffectorSphere = current;
                    _joints = bones.ToArray();
                    _base = _joints[0];

                    SetAxis();

                    theta = new float[_joints.Length];
                    initRotation = new Vector3[_joints.Length];
                    for (int i = 0; i < theta.Length; i++)
                    {
                        initRotation[i] = _joints[i].localEulerAngles;
                        theta[i] = 0;
                        SetDefaultAngle(theta[i], i);
                    }

                    startOffset = new Vector3[_joints.Length];
                    for (int i = 1; i < startOffset.Length; i++)
                        startOffset[i] = _joints[i].position - _joints[i - 1].position;

                    for (int i = 0; i < theta.Length; i++)
                    {
                        if (axis[i] == Vector3.forward)
                        {                            
                            theta[i] = initRotation[i].z;
                        }
                        else
                        {
                            theta[i] = initRotation[i].x;
                        }
                        _joints[i].localEulerAngles = initRotation[i];
                    }
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
                    _joints = bones.ToArray();

                    _base = _joints[0];
                    theta = new float[_joints.Length];
                    for (int i = 0; i < _joints.Length; i++)
                        theta[i] = 0;
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    break;
            }
            return Joints;
        }

        void SetAxis()
        {
            axis = new Vector3[]
            {
                Vector3.forward,
                Vector3.forward,
                Vector3.right,
                Vector3.right,
                Vector3.right,
                Vector3.right
            };
        }

        void SetDefaultAngle(float angle, int i)
        {
            _joints[i].localEulerAngles = axis[i] * angle;
        }
    }
}
