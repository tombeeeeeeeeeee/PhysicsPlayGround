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
    public Vertex[] verticies;
    public float radius;
    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }
}
