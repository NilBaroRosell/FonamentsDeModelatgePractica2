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

        public Transform[] Bones { get => _bones; }

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
                    //TODO: in _endEffectorphere you  keep a reference to the sphere with a collider attached to the endEffector
                    break;
            }
            _bones = bones.ToArray();
            Debug.Log(root.gameObject.name + ": " + _bones.Length + "bones" + ", EndEffector: " + _endEffectorSphere.gameObject.name);
            return Bones;
        }
    }
}
