using System;
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
        float minDistance = 0.02f;
        float delta = 0.1f;
        float learningRate = 50f;

        //LEGS
        MyTentacleController[] _legs = new MyTentacleController[6];


        #region public
        public void InitLegs(Transform[] LegRoots, Transform[] LegFutureBases, Transform[] LegTargets)
        {
            _legs = new MyTentacleController[LegRoots.Length];
            //Legs init
            for (int i = 0; i < LegRoots.Length; i++)
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
            //TODO: Initialize anything needed for the Gradient Descent implementation
        }

        //TODO: Check when to start the animation towards target and implement Gradient Descent method to move the joints.
        public void NotifyTailTarget(Transform target)
        {
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
            if (Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position) < animationRange)
            {
                update_gradient();
            }
        }
        //TODO: implement fabrik method to move legs 
        private void updateLegs()
        {

        }

        void update_gradient()
        {
            //If the distance between the end effector and the tails' target is larger or equal to minDistance 
            if (Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position) >= minDistance)
            {
                ApproachTarget(tailTarget.position);
            }
        }

        void update_fabrik()
        {

        }
        #endregion

        //Apply the new rotations
        void SetAngle(float angle, int i)
        {
            //Apply the new rotation
            _tail.Joints[i].localEulerAngles = _tail.Axis[i] * angle;
            //If the rotation axis is the forward one
            if (_tail.Axis[i] == Vector3.forward)
            {
                //Apply the initial rotation in the X axis
                _tail.Joints[i].localEulerAngles += new Vector3(_tail.InitRotation[i].x, 0, 0);
            }
        }

        //Get the endEffector's position and draw all the joints
        public Vector3 ForwardKinematics()
        {
            Vector3 prevPoint = _tail.Base.position;
            Quaternion rotation = Quaternion.identity;

            for (int i = 1; i < _tail.Joints.Length; i++)
            {
                //Take the initial position of the previous point into account
                rotation *= Quaternion.AngleAxis(_tail.Theta[i - 1], _tail.Axis[i - 1]);
                if (_tail.Axis[i - 1] == Vector3.forward)
                {
                    rotation *= Quaternion.AngleAxis(_tail.InitRotation[i - 1].x, Vector3.right);
                }
                //Calculate the next point
                Vector3 nextPoint = prevPoint + rotation * _tail.StartOffset[i];
                Debug.DrawLine(prevPoint, nextPoint, Color.blue);
                //Set the previous point to the next point
                prevPoint = nextPoint;
            }

            return prevPoint;
        }

        //Calculate the distance between the end effector and the target
        public float DistanceFromTarget(Vector3 target)
        {
            Vector3 point = ForwardKinematics();
            return Vector3.Distance(point, target);
        }

       //Calculates the gradient
        public float CalculateGradient(Vector3 target, int num)
        {
            //Store the current angle
            float solutionAngle = _tail.Theta[num];
            //Calculate the distance to the target
            float f_x = DistanceFromTarget(target);
            //Adds the delta to the angle
            _tail.Theta[num] += delta;
            //Calculates the new distance to the target
            float f_x_plus_h = DistanceFromTarget(target);
            //Calculates the gradient value
            float gradient = (f_x_plus_h - f_x) / delta;
            //Restore the initial current angle
            _tail.Theta[num] = solutionAngle;

            return gradient;
        }

        //Rotate the joints to aproach the target
        public void ApproachTarget(Vector3 target)
        {
            //Calculate the gradient for each joint
            for (int i = 0; i < _tail.Joints.Length - 1; i++)
            {
                _tail.Theta[i] -= learningRate * CalculateGradient(target, i);
            }
            //Apply the new rotation for each joint
            for (int i = 0; i < _tail.Joints.Length - 1; i++)
                SetAngle(_tail.Theta[i], i);
        }
    }
}
