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

        //Aquesta funció retorna la descomposició del swing i twist del Quaternion rotation en un deteminat eix direction
        void swing_twist_decomposition(Quaternion rotation, Vector3 direction, out Quaternion swing, out Quaternion twist)
        {
            //Calculem l'eix de rotació rotationAxis
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            //Calculem la projecció del vector de l'eix de rotacio rotationAxis al eix direction
            Vector3 pos = Vector3.Project(rotationAxis, direction);

            //Descomposem el twist i el swing
            twist = new Quaternion(pos.x, pos.y, pos.z, rotation.w);
            twist = twist.normalized;
            swing = rotation * Quaternion.Inverse(twist);
        }

        void update_ccd()
        {
            //Recorrem tots els tentacles
            for (int i = 0; i < _tentacles.Length; i++)
            {
                //Ens guardem la posició del target
                Vector3 targetPosition = _randomTargets[i].position;

                //Si el tentacle que estem recorrent és el que es troba la pilota i hem rebut el input de xutar
                if (i == activatedTentacle && activateTentacle)
                {
                    //Posem que el target sigui on xutem la pilota
                    targetPosition = Vector3.Lerp(_tentacles[i].EndEffector.position, _target.position, (Time.time - shotTime) / shotDuration);
                    if ((Time.time - shotTime) / shotDuration >= 1)
                    {
                        activateTentacle = false;
                    }
                }

                //Reiniciem el CCD si encara no estem a la mateixa posició que el target
                if (_tentacles[i].TargetPositionCCD != targetPosition)
                {
                    _tentacles[i].TargetPositionCCD = targetPosition;
                    _tentacles[i].CurrentTries = 0;
                }

                //Comprovem si la distància entre l'end effector i el target es es inferior a minDistance
                //Si la distància és major que minDistance
                if (!(Vector3.Distance(_tentacles[i].EndEffector.position, targetPosition) < minDistance))
                {
                    if (_tentacles[i].CurrentTries <= maxTriesCCD)
                    {
                        for (int j = _tentacles[i].Joints.Length - 2; j >= 0; j--)
                        {
                            //Vector desde la posició del current joint fins al end effector
                            Vector3 currentJointToEnd = _tentacles[i].EndEffector.position - _tentacles[i].Joints[j].position;
                            //Vector desde current joint fins al target
                            Vector3 currentJointToTarget = targetPosition - _tentacles[i].Joints[j].position;

                            //Eix de rotació
                            Vector3 rotationAxis = Vector3.Cross(currentJointToEnd, currentJointToTarget);

                            //Comprovem que els vectors no siguin paral·lels o molt semblants
                            if (Vector3.Angle(currentJointToEnd, currentJointToTarget) < 175.0f)
                            {
                                rotationAxis.Normalize();

                                //Calculem l'angle desitjat
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

                                //No apliquem constraints
                                if (maxSwing > 45)
                                {
                                    _tentacles[i].Joints[j].rotation = quat * _tentacles[i].Joints[j].rotation;
                                }
                                //Apliquem constraints
                                else
                                {
                                    //Descomposem la rotació calculada
                                    swing_twist_decomposition(quat, _tentacles[i].Joints[j].up, out swing, out twist);

                                    //Si el current join té un parent (no és la base)
                                    if (j != 0)
                                    {
                                        ///CONSTRAINTS SWING
                                        // Calculem la rotació que fariem sense restriccions
                                        Quaternion auxQuat = swing * _tentacles[i].Joints[j].rotation;

                                        // Mirem l'angle entre aquesta rotació sense restriccions i la rotació del parent
                                        float angle = Quaternion.Angle(auxQuat, _tentacles[i].Joints[j - 1].rotation);
                                        if (angle < 1)
                                            angle = 1;

                                        // Rotem el joint per facilitar trobar l'eix de rotació
                                        _tentacles[i].Joints[j].rotation = swing * _tentacles[i].Joints[j].rotation;
                                        Vector3 auxAxis = Vector3.Cross(_tentacles[i].Joints[j - 1].up, _tentacles[i].Joints[j].up);

                                        // Revertim la rotació anterior un cop hem guardat l'eix
                                        _tentacles[i].Joints[j].rotation = Quaternion.Inverse(swing) * _tentacles[i].Joints[j].rotation;

                                        // Apliquem la restricció d'angle
                                        angle = Mathf.Clamp(angle, minSwing, maxSwing);
                                        Quaternion newSwing = Quaternion.AngleAxis(angle, auxAxis);


                                        ///CONSTRAINTS TWIST
                                        // Calculem la rotació que fariem sense restriccions
                                        _tentacles[i].Joints[j].rotation = twist * _tentacles[i].Joints[j].rotation;

                                        //Rotem fins a la rotació del parent i ens quedem només amb el swing
                                        float twistAngle = Vector3.Angle(_tentacles[i].Joints[j].up, _tentacles[i].Joints[j - 1].up);
                                        Vector3 twistAxis = Vector3.Cross(_tentacles[i].Joints[j].up, _tentacles[i].Joints[j - 1].up);
                                        Quaternion twistRotation = Quaternion.AngleAxis(twistAngle, twistAxis);
                                        Quaternion twistAuxQuat, swingAuxQuat;
                                        swing_twist_decomposition(twistRotation, _tentacles[i].Joints[j - 1].up, out swingAuxQuat, out twistAuxQuat);
                                        _tentacles[i].Joints[j].rotation = swingAuxQuat * _tentacles[i].Joints[j].rotation;

                                        //Calculem l'angle entre els right
                                        float rightAngle = Vector3.Angle(_tentacles[i].Joints[j].right, _tentacles[i].Joints[j - 1].right);
                                        if (rightAngle < 1)
                                            rightAngle = 1;

                                        //Apliquem restricció d'angle
                                        rightAngle = Mathf.Clamp(rightAngle, minTwist, maxTwist);

                                        //Revertir a la rotació original aplicant el nou twist
                                        twistAuxQuat = Quaternion.AngleAxis(rightAngle, _tentacles[i].Joints[j - 1].up);

                                        // Rotem el joint amb l'angle restringit respecte al seu parent
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
