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
        
        MyTentacleController[] _tentacles =new  MyTentacleController[4];

        Transform _currentRegion;
        Transform _target;

        Transform[] _randomTargets = new Transform[4];


        int _tentacleToMove;
        bool moveTentacleToBall;
        bool retireTentacleFromBall;
        float shotDuration = 1.0f;
        float shotTime;

        float minDistance = 0.1f;
        int maxTriesCCD = 10;

        float _twistMin, _twistMax;
        float _swingMin, _swingMax;

        #region public methods
        //DO NOT CHANGE THE PUBLIC METHODS!!

        public float TwistMin { set => _twistMin = value; }
        public float TwistMax { set => _twistMax = value; }
        public float SwingMin {  set => _swingMin = value; }
        public float SwingMax { set => _swingMax = value; }
        

        public void TestLogging(string objectName)
        {

           
            Debug.Log(objectName + " is Paul the Octopus");

            
        }

        public void Init(Transform[] tentacleRoots, Transform[] randomTargets)
        {
            _tentacles = new MyTentacleController[tentacleRoots.Length];

            // foreach (Transform t in tentacleRoots)
            for(int i = 0;  i  < tentacleRoots.Length; i++)
            {
                Debug.Log(tentacleRoots[i].gameObject.name);
                _tentacles[i] = new MyTentacleController();
                _tentacles[i].LoadTentacleJoints(tentacleRoots[i],TentacleMode.TENTACLE);
                //i++;
                //TODO: initialize any variables needed in ccd
            }

            _randomTargets = randomTargets;
            //TODO: use the regions however you need to make sure each tentacle stays in its region

        }

              
        public void NotifyTarget(Transform target, Transform region)
        {
            _currentRegion = region;

            switch (region.name)
            {
                case "region1":
                    _tentacleToMove = 0;
                    break;
                case "region2":
                    _tentacleToMove = 1;
                    break;
                case "region3":
                    _tentacleToMove = 2;
                    break;
                case "region4":
                    _tentacleToMove = 3;
                    break;
            }

            _target = target;
        }

        public void NotifyShoot() {
            //TODO. what happens here?
            if (moveTentacleToBall)
            {
                retireTentacleFromBall = true;
            }
            moveTentacleToBall = true;
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

        /*void update_ccd() {
           

        }*/

        //Aquesta funció retorna la descomposició del swing i twist del Quaternion rotation en un deteminat eix direction
        void swing_twist_decomposition(Quaternion rotation, Vector3 direction, out Quaternion swing, out Quaternion twist)
        {
            //Calculem l'eix de rotació rotationAxis
            Vector3 rotationAxis = new Vector3(rotation.x, rotation.y, rotation.z);
            //Calculem la projecció del vector de l'eix de rotacio rotationAxis al eix direction
            Vector3 p = Vector3.Project(rotationAxis, direction);

            //Descomposem el twist i el swing
            twist = new Quaternion(p.x, p.y, p.z, rotation.w);
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
                if (i == _tentacleToMove && moveTentacleToBall)
                {
                    //Posem que el target sigui on xutem la pilota
                    targetPosition = Vector3.Lerp(_tentacles[i].EndEffector.position, _target.position, (Time.time - shotTime) / shotDuration);
                    if ((Time.time - shotTime) / shotDuration >= 1)
                    {
                        moveTentacleToBall = false;
                    }
                }

                //Reiniciem el CCD si encara no estem a la mateixa posició que el target
                if (_tentacles[i].TargetPositionCCD != targetPosition)
                {
                    _tentacles[i].TargetPositionCCD = targetPosition;
                    _tentacles[i].CurrentTries = 0;
                }

                //Comprovem si la distància entre l'end effector i el target es es inferior a minDistance
                bool done = Vector3.Distance(_tentacles[i].EndEffector.position, targetPosition) < minDistance;

                //Si la distància és major que minDistance
                if (!done)
                {
                    if (_tentacles[i].CurrentTries <= maxTriesCCD)
                    {
                        for (int j = _tentacles[i].Bones.Length - 2; j >= 0; j--)
                        {
                            //Vector desde la posició del current joint fins al end effector
                            Vector3 currentToEnd = _tentacles[i].EndEffector.position - _tentacles[i].Bones[j].position;
                            //Vector desde current joint fins al target
                            Vector3 currentToTarget = targetPosition - _tentacles[i].Bones[j].position;

                            //Eix de rotació
                            Vector3 axis = Vector3.Cross(currentToEnd, currentToTarget);

                            //Comprovem que els vectors no siguin paral·lels o molt semblants
                            if (Vector3.Angle(currentToEnd, currentToTarget) < 175.0f)
                            {
                                axis.Normalize();

                                //Calculem l'angle desitjat
                                if (currentToEnd.magnitude * currentToTarget.magnitude <= 0.001f)
                                    Debug.Log("ERROR");
                                else
                                    _tentacles[i].Theta[j] = Mathf.Acos(Vector3.Dot(currentToEnd, currentToTarget) / (currentToEnd.magnitude * currentToTarget.magnitude));

                                if (Mathf.Sin(_tentacles[i].Theta[j]) < 0)
                                    _tentacles[i].Theta[j] = 2 * Mathf.PI + _tentacles[i].Theta[j];

                                _tentacles[i].Theta[j] = Mathf.Clamp(_tentacles[i].Theta[j], -Mathf.PI, Mathf.PI);
                                _tentacles[i].Theta[j] *= Mathf.Rad2Deg;

                                Quaternion swing, twist;
                                Quaternion q = Quaternion.AngleAxis(_tentacles[i].Theta[j], axis);

                                //No apliquem constraints
                                if (_swingMax > 45)
                                {
                                    _tentacles[i].Bones[j].rotation = q * _tentacles[i].Bones[j].rotation;
                                }
                                //Apliquem constraints
                                else
                                {
                                    //Descomposem la rotació calculada
                                    swing_twist_decomposition(q, _tentacles[i].Bones[j].up, out swing, out twist);

                                    //Si el current join té un parent (no és la base)
                                    if (j != 0)
                                    {
                                        ///CONSTRAINTS SWING
                                        // Calculem la rotació que fariem sense restriccions
                                        Quaternion qAux = swing * _tentacles[i].Bones[j].rotation;

                                        // Mirem l'angle entre aquesta rotació sense restriccions i la rotació del parent
                                        float angle = Quaternion.Angle(qAux, _tentacles[i].Bones[j - 1].rotation);
                                        if (angle < 1)
                                            angle = 1;

                                        // Rotem el joint per facilitar trobar l'eix de rotació
                                        _tentacles[i].Bones[j].rotation = swing * _tentacles[i].Bones[j].rotation;
                                        Vector3 auxAxis = Vector3.Cross(_tentacles[i].Bones[j - 1].up, _tentacles[i].Bones[j].up);

                                        // Revertim la rotació anterior un cop hem guardat l'eix
                                        _tentacles[i].Bones[j].rotation = Quaternion.Inverse(swing) * _tentacles[i].Bones[j].rotation;

                                        // Apliquem la restricció d'angle
                                        angle = Mathf.Clamp(angle, _swingMin, _swingMax);
                                        Quaternion newSwing = Quaternion.AngleAxis(angle, auxAxis);


                                        ///CONSTRAINTS TWIST
                                        // Calculem la rotació que fariem sense restriccions
                                        _tentacles[i].Bones[j].rotation = twist * _tentacles[i].Bones[j].rotation;

                                        //Rotem fins a la rotació del parent i ens quedem només amb el swing
                                        float tAngle = Vector3.Angle(_tentacles[i].Bones[j].up, _tentacles[i].Bones[j - 1].up);
                                        Vector3 tAxis = Vector3.Cross(_tentacles[i].Bones[j].up, _tentacles[i].Bones[j - 1].up);
                                        Quaternion tRot = Quaternion.AngleAxis(tAngle, tAxis);
                                        Quaternion tAux, sAux;
                                        swing_twist_decomposition(tRot, _tentacles[i].Bones[j - 1].up, out sAux, out tAux);
                                        _tentacles[i].Bones[j].rotation = sAux * _tentacles[i].Bones[j].rotation;

                                        //Calculem l'angle entre els right
                                        float rAngle = Vector3.Angle(_tentacles[i].Bones[j].right, _tentacles[i].Bones[j - 1].right);
                                        if (rAngle < 1)
                                            rAngle = 1;

                                        //Apliquem restricció d'angle
                                        rAngle = Mathf.Clamp(rAngle, _twistMin, _twistMax);

                                        //Revertir a la rotació original aplicant el nou twist
                                        tAux = Quaternion.AngleAxis(rAngle, _tentacles[i].Bones[j - 1].up);

                                        // Rotem el joint amb l'angle restringit respecte al seu parent
                                        _tentacles[i].Bones[j].rotation = tAux * newSwing * _tentacles[i].Bones[j - 1].rotation;


                                    }
                                    else
                                    {
                                        _tentacles[i].Bones[j].rotation = swing * _tentacles[i].Bones[j].rotation;
                                    }
                                }


                            }
                            else
                            {
                                Debug.Log("ERROR CROSS = 0");
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
