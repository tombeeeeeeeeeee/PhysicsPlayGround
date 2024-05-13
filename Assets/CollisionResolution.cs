using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.Experimental.AI;
using UnityEngine.UIElements;
using static UnityEditor.PlayerSettings;

public struct CollisionPacket
{
    public Vector3 worldContact;

    public Vector3 normal;
    public Vector3 tangentA; public Vector3 tangentB;
    public float depth;

    public CollisionPacket(float _depth = 0)
    {
        depth = _depth;
        worldContact = Vector3.zero;
        normal = Vector3.zero;
        tangentA = tangentB = Vector3.zero;
    }
}

public enum GJKEvolution
{
    evolving = 0,
    intersecting = 1,
    notIntersecting = 2
}

public struct Simplex
{
    public List<Vector3> points;

    public Simplex(int i = 0)
    {
        points = new List<Vector3>();
    }
}

public struct Polytope
{
    List<Vector3> points;
}

public class CollisionResolution : MonoBehaviour
{

    public GameObject[] testingOrbs;

    private Vector3 direction;
    public GameObject[] collidables;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        for (int i = 0; i < collidables.Length - 1; i++)
        {
            for (int j = i + 1; j < collidables.Length; j++)
            {
                GJK(collidables[i], collidables[j]);
            }
        }
    }

    CollisionPacket GJK(GameObject a, GameObject b)
    {
        CollisionPacket collision = new CollisionPacket();
        Simplex simp = new Simplex(0);
        Vector3[] shapeA = a.GetComponent<MeshFilter>().mesh.vertices.Distinct().ToArray();
        Vector3[] shapeB = b.GetComponent<MeshFilter>().mesh.vertices.Distinct().ToArray();

        //Collision Check
        direction = Vector3.right;
        Vector3 support = CalculateSupport(shapeA, a.transform, shapeB, b.transform, direction);
        GJKEvolution gjking = AddSupportToSimplex(ref simp, support);
        direction = -support;
        int iter = 0;
        while (gjking == GJKEvolution.evolving && iter < 100)
        {
            iter++;
            support = CalculateSupport(shapeA, a.transform, shapeB, b.transform, direction);
            if (AddSupportToSimplex(ref simp, support) == GJKEvolution.evolving)
            {
                gjking = EvolveSimplex(ref simp);
            }
            else
            {
                gjking = GJKEvolution.notIntersecting;
            }
        }

        if (iter == 100)
        {
            //TODO ADD CHECK SAT ON SIMPLEX
        }

        //Debuging Collision
        if (gjking == GJKEvolution.intersecting || iter == 100)
        {
            a.GetComponent<Renderer>().material.color = Color.red;
            b.GetComponent<Renderer>().material.color = Color.red;


            List<Vector3> polytope = simp.points;
            List<int> faces = new List<int>
            {
                0, 1, 2,
                0, 3, 1,
                0, 2, 3,
                1, 3, 2
            };

            int minFace = 0;
            List<Vector4> normals = GetFaceNormals(polytope, faces,out minFace);

            Vector3 minNormal = Vector3.up;
            float minDistance = float.MaxValue;

            while (minDistance == float.MaxValue)
            {
                minNormal = new Vector3(normals[minFace].x, normals[minFace].y, normals[minFace].z);
                minDistance = normals[minFace].w;

                Vector3 sup = CalculateSupport(shapeA, a.transform, shapeB, b.transform, minNormal);
                float sDistance = Vector3.Dot(minNormal, support);

                if(Mathf.Abs(sDistance - minDistance) > 0.001f)
                {
                    minDistance = float.MaxValue;
                    List<Tuple<int, int>> uniqueEdges = new List<Tuple<int,int>>();
                    for(int i = 0; i < normals.Count(); i++)
                    {
                        if (Mathf.Abs(Vector3.Dot(normals[i], sup)) <= 0.00001f)
                        {
                            int f = i * 3;
                            AddIfUniqueEdge(ref uniqueEdges, faces, f + 0, f + 1);
                            AddIfUniqueEdge(ref uniqueEdges, faces, f + 1, f + 2);
                            AddIfUniqueEdge(ref uniqueEdges, faces, f + 1, f + 0);

                            faces[f + 2] = faces[faces.Count - 1]; faces.RemoveAt(faces.Count - 1);
                            faces[f + 1] = faces[faces.Count - 1]; faces.RemoveAt(faces.Count - 1);
                            faces[f + 0] = faces[faces.Count - 1]; faces.RemoveAt(faces.Count - 1);

                            normals[i] = normals[normals.Count() - 1];
                            normals.RemoveAt(normals.Count() - 1);
                            i--;
                        }
                    }

                    List<int> newFaces = new List<int>();
                    foreach(Tuple<int,int> edge in uniqueEdges)
                    {
                        newFaces.Add(edge.Item1);
                        newFaces.Add(edge.Item2);
                        newFaces.Add(polytope.Count());
                    }

                    polytope.Add(sup);

                    int newMinFace;
                    List<Vector4> newNormals = GetFaceNormals(polytope, newFaces, out newMinFace);
                    float oldMinDistance = float.MaxValue;
                    for(int i = 0; i < normals.Count(); i++)
                    {
                        if (normals[i].w < oldMinDistance)
                        {
                            oldMinDistance = normals[i].w;
                            minFace = i;
                        }
                    }

                    if (newNormals[newMinFace].w < oldMinDistance)
                    {
                        minFace = newMinFace + normals.Count();
                    }

                    foreach(int edge in newFaces)
                    { 
                        faces.Add(edge);
                    }
                    foreach(Vector4 normal in newNormals)
                    {
                        normals.Add(normal);
                    }
                }
            }

            collision.normal = new Vector3(minNormal.x, minNormal.y, minNormal.z);
            collision.depth = minDistance;

            a.transform.position -= collision.normal * collision.depth;
        }
        else
        {
            a.GetComponent<Renderer>().material.color = Color.white;
            b.GetComponent<Renderer>().material.color = Color.white;
        }

        return collision;
    }

    GJKEvolution EvolveSimplex(ref Simplex simp)
    {
        switch (simp.points.Count)
        {
            case 2:

                // line ab is the line formed by the first two vertices
                Vector3 ab = simp.points[1] - simp.points[0];
                // line a0 is the line from the first vertex to the origin
                Vector3 dO = simp.points[0] * -1;

                if (Vector3.Dot(ab, dO) > 0)
                {
                    // use the triple-cross-product to calculate shapeA direction perpendicular
                    // to line ab in the direction of the origin
                    Vector3 temp = Vector3.Cross(ab, dO);
                    direction = Vector3.Cross(temp, ab);
                }
                else
                {
                    simp.points.RemoveAt(1);
                    direction = dO;
                }

                return GJKEvolution.evolving;

            case 3:
                Vector3 ac = simp.points[2] - simp.points[0];
                ab = simp.points[1] - simp.points[0];
                dO = simp.points[0] * -1;

                direction = Vector3.Cross(ac, ab);

                if (Vector3.Dot(ac, dO) <= 0) direction *= -1;

                break;
            case 4:
                // calculate the three edges of interest
                Vector3 da = simp.points[0] - simp.points[3];
                Vector3 db = simp.points[1] - simp.points[3];
                Vector3 dc = simp.points[2] - simp.points[3];

                // and the direction to the origin
                dO = simp.points[3] * -1;

                // check triangles a-b-d, b-c-d, and c-a-d
                Vector3 abdNorm = Vector3.Cross(da, db);
                Vector3 bcdNorm = Vector3.Cross(db, dc);
                Vector3 cadNorm = Vector3.Cross(dc, da);

                if (Vector3.Dot(bcdNorm, dO) > 0)
                {
                    simp.points.RemoveAt(0);
                    direction = bcdNorm;
                }
                else if (Vector3.Dot(abdNorm, dO) > 0)
                {
                    simp.points.RemoveAt(2);
                    direction = abdNorm;
                }
                else if (Vector3.Dot(cadNorm, dO) > 0)
                {
                    simp.points.RemoveAt(1);
                    direction = cadNorm;
                }
                else
                {
                    // the origin is inside all of the triangles!
                    return GJKEvolution.intersecting;
                }
                break;

            default:
                Debug.Log("Something is broken");
                return GJKEvolution.notIntersecting;
        }
        return GJKEvolution.evolving;
    }

    GJKEvolution AddSupportToSimplex(ref Simplex simp, Vector3 vert)
    {
        simp.points.Add(vert);
        return Vector3.Dot(direction, vert) >= 0 ? GJKEvolution.evolving : GJKEvolution.notIntersecting;
    }

    Vector3 CalculateSupport(Vector3[] a, Transform aTransform, Vector3[] b, Transform bTransform, Vector3 dir)
    {
        Vector3 localContactA = SupportFunction(aTransform.InverseTransformVector(dir), a);
        Vector3 localContactB = SupportFunction(bTransform.InverseTransformVector(-dir), b);

        Vector3 worldContactA = aTransform.TransformPoint(localContactA);
        Vector3 worldContactB = bTransform.TransformPoint(localContactB);
        return worldContactA - worldContactB;
    }

    Vector3 SupportFunction(Vector3 dir, Vector3[] vecs)
    {
        float max = float.NegativeInfinity;
        int index = 0;
        for (int i = 0; i < vecs.Length; i++)
        {
            float dot = Vector3.Dot(vecs[i], dir);
            if (dot > max)
            {
                max = dot;
                index = i;
            }
        }
        return vecs[index];
    }

    List<Vector4> GetFaceNormals(List<Vector3> polytope, List<int> faces, out int face)
    {
        List<Vector4> normals = new List<Vector4>();

        int minTriangle = 0;
        float minDistance = float.MaxValue;
        for(int i = 0; i < faces.Count; i+=3)
        {
            Vector3 a = polytope[faces[i]];
            Vector3 b = polytope[faces[i+1]];
            Vector3 c = polytope[faces[i+2]];

            Vector3 normal = Vector3.Normalize(Vector3.Cross(b-a,c-a));
            float distance = Vector3.Dot(normal, a);

            if(distance < 0 )
            {
                normal *= -1;
                distance *= -1;
            }
            Vector4 packet = new Vector4(normal.x, normal.y, normal.z, distance);
            normals.Add(packet);

            if(distance < minDistance)
            {
                minTriangle = i / 3;
                minDistance = distance;
            }
        }
        face = minTriangle;
        return normals;
    }

    void AddIfUniqueEdge(ref List<Tuple<int, int>> edges, List<int> faces, int aVert, int bVert)
    {
        bool contains = edges.Contains(new Tuple<int, int>(faces[bVert], faces[aVert]) );
        if (contains)
        {
            edges.Remove(new Tuple<int, int>(faces[bVert], faces[aVert]));
        }
        else
        {
            edges.Add(new Tuple<int, int>(faces[aVert], faces[bVert]));
        }
    }
}

