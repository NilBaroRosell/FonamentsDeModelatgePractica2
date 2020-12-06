﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
  
    public class MyScorpionController
    {
        //TAIL
        Transform tailTarget;
        Transform tailEndEffector;
        MyTentacleController _tail;
        float animationRange = 1.0f;
        float minDist = 0.1f;
        float speed = 5;

        //LEGS
        Transform[] legTargets;
        Transform[] legFutureBases;
        MyTentacleController[] _legs = new MyTentacleController[6];

        
        #region public
        public void InitLegs(Transform[] LegRoots,Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            //Legs init
            for(int i = 0; i < LegRoots.Length; i++)
            {
                _legs[i] = new MyTentacleController();
                _legs[i].LoadTentacleJoints(LegRoots[i], TentacleMode.LEG);
                //TODO: initialize anything needed for the FABRIK implementation
            }

        }

        public void InitTail(Transform TailBase)
        {
            _tail = new MyTentacleController();
            _tail.LoadTentacleJoints(TailBase, TentacleMode.TAIL);
            tailEndEffector = _tail.EndEffector;
            Vector3 targetPosition = _tail.ForwardKinematics();
            /*targetPosition = Vector3.Lerp(_tail.EndEffector.position, targetPosition, Time.deltaTime * speed);
            _tail.ApproachTarget(targetPosition);*/
            //TODO: Initialize anything needed for the Gradient Descent implementation
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
            //guardar posicio de la bola
            tailTarget = target;
        }

        //TODO: Notifies the start of the walking animation
        public void NotifyStartWalk()
        {

        }

        //TODO: create the apropiate animations and update the IK from the legs and tail

        public void UpdateIK()
        {
            updateTail();
        }
        #endregion


        #region private
        //TODO: Implement the leg base animations and logic
        private void updateLegPos()
        {
            //check for the distance to the futureBase, then if it's too far away start moving the leg towards the future base position
            //
        }
        //TODO: implement Gradient Descent method to move tail if necessary
        private void updateTail()
        {
            //Debug.Log(tailTarget.position);

            //Debug.Log(Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position));
            //Debug.Log(Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position) < animationRange);
            if (Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position) < animationRange)
            {
                //Debug.Log("ENTRAAAAAAAAAAAAAAAAA");
                update_gradient();
            }
            //si la posicio de la bola respecte l'endeffector de la cua es menor que tailDistance
            //update gradient decent
        }
        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {

        }

        void update_gradient()
        {
            Vector3 targetPosition = Vector3.Lerp(_tail.EndEffector.position, tailTarget.position, Time.deltaTime * speed);

            //Debug.Log(targetPosition);

            //Comprovem si la distància entre l'end effector i el target es es inferior a minDistance
            bool done = Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position) < minDist;

            if (!done)
            {
                _tail.ApproachTarget(targetPosition);
            }
        }

        void update_fabrik()
        {

        }
        #endregion
    }
}
