using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UIElements;
using static UnityEditor.PlayerSettings;

public enum GJKEvolution
{
    evolving = 0,
    intersecting = 1,
    notIntersecting = 2
}

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

public struct Simplex
{
    public List<Vector3> points;

    public Simplex(int i = 0)
    {
        points = new List<Vector3>();
    }
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
        for(int i = 0; i < collidables.Length - 1; i++)
        {
            for(int j = i + 1; j < collidables.Length; j++)
            {
                CollisionCheck(collidables[i], collidables[j]);
            }
        }
    }

    CollisionPacket CollisionCheck(GameObject a, GameObject b)
    {
        CollisionPacket collision = new CollisionPacket();
        Simplex simp = new Simplex(0);
        Vector3[] shapeA = a.GetComponent<MeshFilter>().mesh.vertices.Distinct().ToArray();
        Vector3[] shapeB = b.GetComponent<MeshFilter>().mesh.vertices.Distinct().ToArray();

       //for (int i = 0; i < shapeB.Length; i++)
       //{
       //    shapeB[i] = b.transform.TransformPoint(shapeB[i]);
       //    testingOrbs[i].transform.position = shapeB[i];
       //}

        

        //WorldToLocal
        //for (int i = 0; i < shapeB.Length; i++)
        //{
        //    shapeB[i] = Matrix4x4.Rotate(b.transform.rotation) * shapeB[i];
        //    shapeB[i] += b.transform.position;
        //}

        //Collision Check
        direction = Vector3.right;
        Vector3 support = CalculateSupport(shapeA, a.transform, shapeB, b.transform);
        GJKEvolution gjking = AddSupportToSimplex(ref simp, support);
        direction = -support;
        int iter = 0;
        while(gjking == GJKEvolution.evolving && iter < 100)
        {
            iter++;
            support = CalculateSupport(shapeA, a.transform, shapeB, b.transform);
            if (AddSupportToSimplex(ref simp, support) == GJKEvolution.evolving)
            {
                gjking = EvolveSimplex(ref simp, shapeA, shapeB, ref collision);
            }
            else
            {
                gjking = GJKEvolution.notIntersecting;
            }
        }

        if(iter == 100)
        {
            //TODO ADD CHECK SAT ON SIMPLEX
        }

        //Debuging Collision
        if(gjking == GJKEvolution.intersecting)
        {
            //a.GetComponent<Renderer>().material.color = Color.red;
            //b.GetComponent<Renderer>().material.color = Color.red;
            collision.normal = -Vector3.Normalize(Vector3.Dot(collision.normal, b.transform.position - a.transform.position) * collision.normal);
            testingOrbs[0].transform.position = collision.worldContact;
            a.transform.position += collision.normal * collision.depth;
            Debug.Log("Normal is: " + collision.normal);
            Debug.Log("depth is: " + collision.depth);
        }
        else
        {
            a.GetComponent<Renderer>().material.color = Color.white;
            b.GetComponent<Renderer>().material.color = Color.white;
        }

        

        return collision;
    }

    GJKEvolution EvolveSimplex(ref Simplex simp, Vector3[] a, Vector3[] b, ref CollisionPacket collision)
    {
        switch (simp.points.Count)
        {
            case 2:

                // line ab is the line formed by the first two vertices
                Vector3 ab = simp.points[1] - simp.points[0];
                // line a0 is the line from the first vertex to the origin
                Vector3  dO = simp.points[0] * -1;

                if(Vector3.Dot(ab, dO) > 0)
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
                    // the origin is on the outside of triangle bcd
                    // eliminate d!
                    simp.points.RemoveAt(0);
                    direction = bcdNorm;
                }
                else if (Vector3.Dot(abdNorm, dO) > 0)
                {
                    // the origin is on the outside of triangle a-b-d
                    // eliminate c!
                    simp.points.RemoveAt(2);
                    direction = abdNorm;
                }
                else if (Vector3.Dot(cadNorm, dO) > 0)
                {
                    // the origin is on the outside of triangle cad
                    // eliminate b!
                    simp.points.RemoveAt(1);
                    direction = cadNorm;
                }
                else
                {
                    float depth = -Vector3.Dot(bcdNorm, dO);
                    Vector3 normal = -bcdNorm;

                    float abdDepth = Vector3.Dot(-abdNorm, dO);
                    if (abdDepth < depth)
                    {
                        normal = abdNorm;
                        depth = abdDepth;
                    }

                    float cadDepth = Vector3.Dot(-cadNorm, dO);
                    if (cadDepth < depth)
                    {
                        normal = cadNorm;
                        depth = cadDepth;
                    }

                    ab = simp.points[1] - simp.points[0];
                    ac = simp.points[2] - simp.points[0];
                    Vector3 abcNorm = Vector3.Cross(ab, ac);
                    float abcDepth = Vector3.Dot(-abcNorm, dO);
                    if (abcDepth < depth)
                    {
                        normal = abcNorm;
                        depth = abcDepth;
                    }

                    collision.normal = normal;
                    collision.depth = depth;

                    

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

    Vector3 CalculateSupport(Vector3[] a, Transform aTransform, Vector3[] b, Transform bTransform)
    {
        Vector3 localContactA = SupportFunction(aTransform.InverseTransformVector(direction), a);
        Vector3 localContactB = SupportFunction(bTransform.InverseTransformVector(-direction), b);

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

    void CalculateCollisionPoint(Vector3[] a, Transform aTransform, Vector3[] b, Transform bTransform, ref CollisionPacket collision)
    {
        Vector3 localA = SupportFunction(aTransform.InverseTransformVector(direction), a);
        Vector3 localB = SupportFunction(bTransform.InverseTransformVector(-direction), b);

        Vector3 worldA = aTransform.TransformPoint(localA);
        Vector3 worldB = bTransform.TransformPoint(localB);

        if (Vector3.Dot(Vector3.Normalize(worldA), direction) > Vector3.Dot(Vector3.Normalize(worldB), direction))
        {
            collision.worldContact = worldA;
        }
        else
        {
            collision.worldContact = worldB;
        }
    }
}
