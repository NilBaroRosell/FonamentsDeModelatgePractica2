using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;


namespace OctopusController
{
    public enum TentacleMode { LEG, TAIL, TENTACLE };

    public class MyOctopusController
    {

        MyTentacleController[] _tentacles = new MyTentacleController[4];
        Transform _target;

        Transform[] _randomTargets = new Transform[4];


        int activatedTentacle;
        bool activateTentacle;
        float shotDuration = 3.0f;
        float shotTime;

        float minDistance = 0.1f;
        int maxTriesCCD = 10;

        float minTwist;
        float maxTwist;
        float minSwing;
        float maxSwing;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => minTwist = 0; }
        public float TwistMax { set => maxTwist = 20; }
        public float SwingMin { set => minSwing = 0; }
        public float SwingMax { set => maxSwing = 8; }


        public void TestLogging(string objectName)
        {
            Debug.Log(objectName + "is Paul the Octopus");
        }

        public void Init(Transform[] tentacleRoots, Transform[] randomTargets)
        {
            _tentacles = new MyTentacleController[tentacleRoots.Length];

            // foreach (Transform t in tentacleRoots)
            for (int i = 0; i < tentacleRoots.Length; i++)
            {
                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i], TentacleMode.TENTACLE);
                //TODO: initialize any variables needed in ccd
            }

            _randomTargets = randomTargets;
            //TODO: use the regions however you need to make sure each tentacle stays in its region

        }


        public void NotifyTarget(Transform target, Transform region)
        {
            switch (region.name)
            {
                case "region1":
                    activatedTentacle = 0;
                    break;
                case "region2":
                    activatedTentacle = 1;
                    break;
                case "region3":
                    activatedTentacle = 2;
                    break;
                case "region4":
                    activatedTentacle = 3;
                    break;
            }

            _target = target;
        }

        public void NotifyShoot()
        {
            //TODO. what happens here?
            activateTentacle = true;
            shotTime = Time.time;
        }


        public void UpdateTentacles()
        {
            //TODO:   logic for the correct tentacle arm to stop the ball and implement CCD method
            update_ccd();
        }




        #endregion


        #region private and internal methods
        //todo: add here anything that you need

        //Returs the decomposed swing and twist of the quaternion rotation in a specific direction 
        void decomposeSwingAndTwist(Quaternion rotation, Vector3 direction, out Quaternion swing, out Quaternion twist)
        {
            //Calculate the rotation axis
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);

            //Calculate the projection of the axis rotation in the direction axis
            Vector3 pos = Vector3.Project(rotationAxis, direction);

            //Decompose the twist and the swing
            twist = new Quaternion(pos.x, pos.y, pos.z, rotation.w);
            twist = twist.normalized;
            swing = rotation * Quaternion.Inverse(twist);
        }

        void update_ccd()
        {
            //Loop through the tentacles
            for (int i = 0; i < _tentacles.Length; i++)
            {
                //Store target's position
                Vector3 targetPosition = _randomTargets[i].position;

                //If the current tentacle is in the same region as the ball and we recieved the shooting input
                if (i == activatedTentacle && activateTentacle)
                {
                    //Set the target's position to the ball's target
                    targetPosition = Vector3.Lerp(_tentacles[i].EndEffector.position, _target.position, (Time.time - shotTime) / shotDuration);
                    if ((Time.time - shotTime) / shotDuration >= 1)
                    {
                        activateTentacle = false;
                    }
                }

                //Restart CCD if the tentacle is not in the same position as its target
                if (_tentacles[i].TargetPositionCCD != targetPosition)
                {
                    _tentacles[i].TargetPositionCCD = targetPosition;
                    _tentacles[i].CurrentTries = 0;
                }

                //If the distance between the end effector and the tentacle's target is larger or equal to minDistance 
                if (Vector3.Distance(_tentacles[i].EndEffector.position, targetPosition) >= minDistance)
                {
                    if (_tentacles[i].CurrentTries <= maxTriesCCD)
                    {
                        for (int j = _tentacles[i].Joints.Length - 2; j >= 0; j--)
                        {
                            //Store distance between current joint and end effector
                            Vector3 currentJointToEnd = _tentacles[i].EndEffector.position - _tentacles[i].Joints[j].position;
                            //Store distance between current joint and target
                            Vector3 currentJointToTarget = targetPosition - _tentacles[i].Joints[j].position;

                            //Store rotation axis
                            Vector3 rotationAxis = Vector3.Cross(currentJointToEnd, currentJointToTarget);

                            //If the vectors ar not paralel or very similar
                            if (Vector3.Angle(currentJointToEnd, currentJointToTarget) < 175.0f)
                            {
                                rotationAxis.Normalize();

                                //Calculate the desired angle
                                if (currentJointToEnd.magnitude * currentJointToTarget.magnitude <= 0.001f)
                                    Debug.Log("ERROR");
                                else
                                    _tentacles[i].Theta[j] = Mathf.Acos(Vector3.Dot(currentJointToEnd, currentJointToTarget) / (currentJointToEnd.magnitude * currentJointToTarget.magnitude));

                                if (Mathf.Sin(_tentacles[i].Theta[j]) < 0)
                                    _tentacles[i].Theta[j] = 2 * Mathf.PI + _tentacles[i].Theta[j];

                                _tentacles[i].Theta[j] = Mathf.Clamp(_tentacles[i].Theta[j], -Mathf.PI, Mathf.PI);
                                _tentacles[i].Theta[j] *= Mathf.Rad2Deg;

                                Quaternion swing, twist;
                                Quaternion quat = Quaternion.AngleAxis(_tentacles[i].Theta[j], rotationAxis);

                                if (maxSwing > 45)
                                {
                                    _tentacles[i].Joints[j].rotation = quat * _tentacles[i].Joints[j].rotation;
                                }
                                else
                                {
                                    //Decompose the calculated rotation
                                    decomposeSwingAndTwist(quat, _tentacles[i].Joints[j].up, out swing, out twist);

                                    //If the current joint has a parent (ergo it's not the base)
                                    if (j != 0)
                                    {
                                        ///SWING
                                        //Calculate the rotation without constraints
                                        Quaternion auxQuat = swing * _tentacles[i].Joints[j].rotation;

                                        //Get the angle between this rotation and the parent's rotation
                                        float angle = Quaternion.Angle(auxQuat, _tentacles[i].Joints[j - 1].rotation);
                                        if (angle < 1)
                                            angle = 1;

                                        //Rotate the joint to easily find the rotation axis
                                        _tentacles[i].Joints[j].rotation = swing * _tentacles[i].Joints[j].rotation;
                                        Vector3 auxAxis = Vector3.Cross(_tentacles[i].Joints[j - 1].up, _tentacles[i].Joints[j].up);

                                        //Go back to the previous rotation once we've stored the axis
                                        _tentacles[i].Joints[j].rotation = Quaternion.Inverse(swing) * _tentacles[i].Joints[j].rotation;

                                        //Apply constraints to the angle
                                        angle = Mathf.Clamp(angle, minSwing, maxSwing);
                                        Quaternion newSwing = Quaternion.AngleAxis(angle, auxAxis);


                                        ///TWIST
                                        //Calculate the rotation without constraints
                                        _tentacles[i].Joints[j].rotation = twist * _tentacles[i].Joints[j].rotation;

                                        //Rotate until we get the parent's rotation and we store the swing
                                        float twistAngle = Vector3.Angle(_tentacles[i].Joints[j].up, _tentacles[i].Joints[j - 1].up);
                                        Vector3 twistAxis = Vector3.Cross(_tentacles[i].Joints[j].up, _tentacles[i].Joints[j - 1].up);
                                        Quaternion twistRotation = Quaternion.AngleAxis(twistAngle, twistAxis);
                                        Quaternion twistAuxQuat, swingAuxQuat;
                                        decomposeSwingAndTwist(twistRotation, _tentacles[i].Joints[j - 1].up, out swingAuxQuat, out twistAuxQuat);
                                        _tentacles[i].Joints[j].rotation = swingAuxQuat * _tentacles[i].Joints[j].rotation;

                                        //Calculate the angle between the right vectors
                                        float rightAngle = Vector3.Angle(_tentacles[i].Joints[j].right, _tentacles[i].Joints[j - 1].right);
                                        if (rightAngle < 1)
                                            rightAngle = 1;

                                        //Apply constraints to the angle
                                        rightAngle = Mathf.Clamp(rightAngle, minTwist, maxTwist);

                                        //Go back to the original rotation applying the new twist
                                        twistAuxQuat = Quaternion.AngleAxis(rightAngle, _tentacles[i].Joints[j - 1].up);

                                        //Rotate the joint with the constrained angle in relation to its parent
                                        _tentacles[i].Joints[j].rotation = twistAuxQuat * newSwing * _tentacles[i].Joints[j - 1].rotation;


                                    }
                                    else
                                    {
                                        _tentacles[i].Joints[j].rotation = swing * _tentacles[i].Joints[j].rotation;
                                    }
                                }


                            }
                        }

                        _tentacles[i].CurrentTries++;
                    }
                }
            }

        }



        #endregion






    }
}
