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
        Vector3[] initRotation;

        float delta = 0.1f;
        float learningRate = 50f;

        float[] theta;
        Vector3 targetPositionCCD;
        int currentTries;

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
                    _bones = bones.ToArray();

                    _base = _bones[0];
                    theta = new float[_bones.Length];
                    for (int i = 0; i < _bones.Length; i++)
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
                    _bones = bones.ToArray();
                    _base = _bones[0];

                    SetAxis();

                    theta = new float[_bones.Length];
                    initRotation = new Vector3[_bones.Length];
                    for (int i = 0; i < theta.Length; i++)
                    {
                        initRotation[i] = _bones[i].localEulerAngles;
                        theta[i] = 0;
                        SetDefaultAngle(theta[i], i);
                    }

                    startOffset = new Vector3[_bones.Length];
                    for (int i = 1; i < startOffset.Length; i++)
                        startOffset[i] = _bones[i].position - _bones[i - 1].position;

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
                        _bones[i].localEulerAngles = initRotation[i];
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
                    _bones = bones.ToArray();

                    _base = _bones[0];
                    theta = new float[_bones.Length];
                    for (int i = 0; i < _bones.Length; i++)
                        theta[i] = 0;
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    break;
            }
            return Bones;
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

        public Vector3 ForwardKinematics()
        {
            Vector3 prevPoint = _base.position;
            Quaternion rotation = Quaternion.identity;

            for (int i = 1; i < _bones.Length; i++)
            {
                rotation *=  Quaternion.AngleAxis(theta[i - 1], axis[i - 1]);
                if(axis[i-1] == Vector3.forward)
                {
                    rotation *= Quaternion.AngleAxis(initRotation[i-1].x, Vector3.right);
                }
                Vector3 nextPoint = prevPoint + rotation * startOffset[i];
                Debug.DrawLine(prevPoint, nextPoint, Color.blue);

                prevPoint = nextPoint;
            }

            return prevPoint;
        }

        public float DistanceFromTarget(Vector3 target)
        {
            Vector3 point = ForwardKinematics();
            return (point - target).magnitude;
        }

        public float CalculateGradient(Vector3 target, int num)
        {
            float auxAngle = theta[num];

            float distance = DistanceFromTarget(target);
            theta[num] += delta;
            float newDistance = DistanceFromTarget(target);

            float gradient = (newDistance - distance) / delta;

            theta[num] = auxAngle;

            return gradient;
        }

        public void ApproachTarget(Vector3 target)
        {
            for (int i = 0; i < _bones.Length - 1; i++)
            {
                theta[i] -= learningRate * CalculateGradient(target, i);
            }

            for (int i = 0; i < _bones.Length - 1; i++)
                SetAngle(theta[i], i);
        }

        void SetDefaultAngle(float angle, int i)
        {
            _bones[i].localEulerAngles = axis[i] * angle;
        }

        void SetAngle(float angle, int i)
        {
            _bones[i].localEulerAngles = axis[i] * angle;
            if(axis[i] == Vector3.forward)
            {
                _bones[i].localEulerAngles += new Vector3(initRotation[i].x, 0, 0);
            }
        }
    }
}
