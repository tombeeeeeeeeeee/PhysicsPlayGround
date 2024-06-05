using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;

[Serializable]
public struct Vertex
{
    public Vector3 vert;
    public int[] edges;
}

[Serializable]
public struct Shape
{
    public Vertex[] vertices;
    public float radius;

    public Vector3[] ToArray()
    {
        Vector3[] verts = new Vector3[vertices.Length];
        for(int i = 0; i < vertices.Length; i++)
        {
            verts[i] = vertices[i].vert;
        }
        return verts;
    }
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
    public float elasticCoef = 0.65f;
    public float drag = 1;
    public float angularDrag = 5;

    // Start is called before the first frame update
    void Start()
    {
        velocity = Vector3.zero;
        netDepen = Vector3.zero;
        angularVelocity = Vector3.zero;
        angularMomentum = Vector3.zero;

        foreach(Shape shape in shapes)
        {
            foreach(Vertex vertex in shape.vertices)
            {
                Vector3 vertWorld = transform.TransformPoint(vertex.vert);
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

        invWorldIT = math.mul(new float3x3(transform.rotation), invBodyIT);
        invWorldIT = math.mul(invWorldIT, math.transpose(invBodyIT));
    }

    // Update is called once per frame
    void Update()
    {

    }
}
