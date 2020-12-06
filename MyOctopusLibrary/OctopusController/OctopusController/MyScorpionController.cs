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
        float minDist = 0.02f;
        float delta = 0.1f;
        float learningRate = 50f;

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
            bool done = Vector3.Distance(tailEndEffector.transform.position, tailTarget.transform.position) < minDist;

            if (!done)
            {
                ApproachTarget(tailTarget.position);
            }
        }

        void update_fabrik()
        {

        }
        #endregion

        void SetAngle(float angle, int i)
        {
            _tail.Joints[i].localEulerAngles = _tail.Axis[i] * angle;
            if (_tail.Axis[i] == Vector3.forward)
            {
                _tail.Joints[i].localEulerAngles += new Vector3(_tail.InitRotation[i].x, 0, 0);
            }
        }


        public Vector3 ForwardKinematics()
        {
            Vector3 prevPoint = _tail.Base.position;
            Quaternion rotation = Quaternion.identity;

            for (int i = 1; i < _tail.Joints.Length; i++)
            {
                rotation *= Quaternion.AngleAxis(_tail.Theta[i - 1], _tail.Axis[i - 1]);
                if (_tail.Axis[i - 1] == Vector3.forward)
                {
                    rotation *= Quaternion.AngleAxis(_tail.InitRotation[i - 1].x, Vector3.right);
                }
                Vector3 nextPoint = prevPoint + rotation * _tail.StartOffset[i];
                Debug.DrawLine(prevPoint, nextPoint, Color.blue);

                prevPoint = nextPoint;
            }

            return prevPoint;
        }

        public float DistanceFromTarget(Vector3 target)
        {
            Vector3 point = ForwardKinematics();
            return Vector3.Distance(point,target);
        }

        public float CalculateGradient(Vector3 target, int num)
        {
            float solutionAngle = _tail.Theta[num];

            float f_x = DistanceFromTarget(target);
            _tail.Theta[num] += delta;
            float f_x_plus_h = DistanceFromTarget(target);

            float gradient = (f_x_plus_h - f_x) / delta;

            _tail.Theta[num] = solutionAngle;

            return gradient;
        }

        public void ApproachTarget(Vector3 target)
        {
            for (int i = 0; i < _tail.Joints.Length - 1; i++)
            {
                _tail.Theta[i] -= learningRate * CalculateGradient(target, i);
            }

            for (int i = 0; i < _tail.Joints.Length - 1; i++)
                SetAngle(_tail.Theta[i], i);
        }
    }
}
