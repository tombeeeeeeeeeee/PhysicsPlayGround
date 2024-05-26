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
    public Vector3 momentOfInertia = new Vector3(3,3,3);

    [HideInInspector]public Vector3 force;
    [HideInInspector]public Vector3 velocity = Vector3.zero;
    [HideInInspector]public Vector3 netDepen;
    [HideInInspector]public float3x3 invBodyIT;
    [HideInInspector]public float3x3 invWorldIT;
    [HideInInspector]public Vector3 torque;
    [HideInInspector]public Vector3 angularVelocity;
    [HideInInspector]public Vector3 angularMomentum;

    private float dX, dY, dZ;
    public List<Shape> shapes;
    public float invMass;
    public Vector3 centreOfMass = Vector3.zero;
    public bool isGravitated = false;
    public float elasticCoef = 1; //0.65f;

    // Start is called before the first frame update
    void Start()
    {
        velocity = Vector3.zero;
        netDepen = Vector3.zero;
        angularVelocity = Vector3.zero;
        angularMomentum = Vector3.zero;

        foreach(Shape shape in shapes)
        {
            foreach(Vector3 vertex in shape.verticies)
            {
                Vector3 vertWorld = transform.TransformPoint(vertex);
                if(Math.Abs(vertWorld.x) + shape.radius > dX) dX = Math.Abs(vertWorld.x) + shape.radius;
                if(Math.Abs(vertWorld.y) + shape.radius > dY) dY = Math.Abs(vertWorld.y) + shape.radius;
                if(Math.Abs(vertWorld.z) + shape.radius > dZ) dZ = Math.Abs(vertWorld.z) + shape.radius;
            }
        }

        dX *= dX; dY *= dY; dZ *= dZ;


        if(invMass > 0) { 
            invBodyIT = new float3x3(
                (momentOfInertia.x * invMass * 12) / (dY + dZ), 0, 0,
                0, (momentOfInertia.y * invMass * 12) / (dX + dZ), 0,
                0, 0, (momentOfInertia.z * invMass * 12) / (dX + dY)
            );
       }

        invWorldIT = math.mul(math.mul(invBodyIT, new float3x3(transform.rotation)), math.transpose(invBodyIT));
        //invWorldIT = math.mul(invBodyIT, math.mul(new float3x3(transform.rotation), math.transpose(invBodyIT)));

    }

    // Update is called once per frame
    void Update()
    {

    }
}
