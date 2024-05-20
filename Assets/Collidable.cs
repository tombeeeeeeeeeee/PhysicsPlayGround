using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;
using static UnityEditor.PlayerSettings;
using Unity.Mathematics;
[Serializable]
public struct Shape
{
    public Vector3[] verticies;
    public float radius;
}

public class Collidable : MonoBehaviour
{
    public Vector3 velocity;
    public Vector3 netDepen;
    public float invMass;
    public float3x3 invBodyIT;
    public float3x3 invWorldIT;
    public float elasticCoef;
    public Vector3 centreOfMass;
    public Vector3 angularVelocity;

    private float dX, dY, dZ;
    public List<Shape> shapes;

    // Start is called before the first frame update
    void Start()
    {
        velocity = Vector3.zero;
        netDepen = Vector3.zero;
        invMass = 0;
        elasticCoef = 0;
        centreOfMass = Vector3.zero;
        angularVelocity = Vector3.zero;
        dX = 0; dY = 0; dZ = 0;
    }

    // Update is called once per frame
    void Update()
    {

    }
}
