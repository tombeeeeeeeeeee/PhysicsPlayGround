using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;
using static UnityEditor.PlayerSettings;


public class Collidable : MonoBehaviour
{
    public Vector3[] verticies;
    public float radius;

    public Vector3 velocity;
    public Vector3 netDepen;
    public float   invMass;
    public float   invMomentOfInertia;
    public float   elasticCoef;
    public Vector3 centreOfMass;
    public Vector3 angularVelocity;

    // Start is called before the first frame update
    void Start()
    {
        velocity = Vector3.zero;
        netDepen = Vector3.zero;
        invMass = 0;
        eladsticCoef = 0;
        centreOfMass = Vector3.zero;
        angularVelocuty = Vector3.zero;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
