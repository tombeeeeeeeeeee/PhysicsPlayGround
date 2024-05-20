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
    [HideInInspector]public Vector3 velocity;
    [HideInInspector]public Vector3 netDepen;
    [HideInInspector]public float3x3 invBodyIT;
    [HideInInspector]public float3x3 invWorldIT;
    [HideInInspector]public float elasticCoef;
    [HideInInspector]public Vector3 angularVelocity;
    [HideInInspector]public Vector3 angularMomentum;

    private float dX, dY, dZ;
    public List<Shape> shapes;
    public float invMass;
    public Vector3 centreOfMass = Vector3.zero;

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
                if(vertex.x + shape.radius > dX) dX = vertex.x + shape.radius;
                if(vertex.y + shape.radius > dY) dY = vertex.y + shape.radius;
                if(vertex.z + shape.radius > dZ) dZ = vertex.z + shape.radius;
            }
        }

        dX *= dX; dY *= dY; dZ *= dZ;

        invBodyIT = new float3x3(
            3 * invMass / (dY + dZ), 0 , 0,
            0, 3 * invMass / (dX + dZ), 0,
            0, 0, 3 * invMass / (dX + dY)
            );

        invWorldIT = math.mul(math.mul(transform.rotation.));

    }

    // Update is called once per frame
    void Update()
    {

    }
}
