using System;
using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;
using static UnityEditor.PlayerSettings;

[Serializable]
public struct Vertex
{
    public Vector3 point;
    public List<int> joinedPoints;
}


public class Collidable : MonoBehaviour
{
    public Transform t; 
    public Vertex[] verticies;
    private List<Vector3> points;
    // Start is called before the first frame update
    void Start()
    {
        t = transform;
        points = new List<Vector3>();
        foreach (Vertex vert in verticies)
            points.Add(vert.point);
    }

    // Update is called once per frame
    void Update()
    {
        for(int i = 0; i < verticies.Length; i++)
        {
            verticies[i].point = (Matrix4x4.Rotate(transform.rotation) * Matrix4x4.Translate(transform.position)) * points[i];
            verticies[i].point += transform.position;
        }
    }
}
