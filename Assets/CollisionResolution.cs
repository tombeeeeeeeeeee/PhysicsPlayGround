using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Mathematics;
using static Unity.Mathematics.math;

public struct CollisionPacket
{
    public Vector3 worldContact;

    public Vector3 normal;
    public float depth;

    public Collidable objectA;
    public Collidable objectB;
    public CollisionPacket(float _depth = 0)
    {
        depth = _depth;
        worldContact = Vector3.zero;
        normal = Vector3.zero;
        objectA = null;
        objectB = null;
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


public class CollisionResolution : MonoBehaviour
{

    public Vector3 gravity = new Vector3(0, -9.8f, 0);
    public GameObject[] testingOrbs;

    public Collidable[] collidables;


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //Intergration Step
        for(int i = 0; i < collidables.Length; i++)
        {
            Intergration(collidables[i]);
        }

        //Collision Checks
        List<CollisionPacket> collisions = new List<CollisionPacket>();

        for(int iter = 0; iter < 1; iter++)
        {
            //BroadPhase
            for (int i = 0; i < collidables.Length - 1; i++)
            {
                for (int j = i + 1; j < collidables.Length; j++)
                {
                    //NarrowPhase
                    if (collidables[i].invMass + collidables[j].invMass != 0)
                    {
                        ShapeCollisionCheck(collidables[i], collidables[j], ref collisions);
                    }
                }
            }
        }

        //Collision Resolution
        foreach(CollisionPacket collision in collisions)
        {
            Resolution(collision);
        }
    }

    private void Intergration(Collidable collidable)
    {
        if (collidable.isGravitated && collidable.invMass != 0)
        {
            collidable.force += gravity / collidable.invMass;
        }

        collidable.velocity *= 1 - 0.001f;
        collidable.velocity += Time.deltaTime * collidable.invMass * collidable.force;
        collidable.angularMomentum *= 1 - 0.1f;
        collidable.angularMomentum += Time.deltaTime * collidable.torque;

        collidable.angularVelocity = math.mul(math.transpose(new float3x3(collidable.transform.rotation)), collidable.angularMomentum);
        collidable.angularVelocity = math.mul(collidable.invBodyIT, collidable.angularVelocity);
        collidable.angularVelocity = math.mul(new float3x3(collidable.transform.rotation), collidable.angularVelocity);

        collidable.transform.position += collidable.netDepen;
        collidable.netDepen = Vector3.zero;

        collidable.transform.position += Time.deltaTime * collidable.velocity;

        //Cross Product Matrix
        float3x3 angVel = new float3x3(
            0, -collidable.angularVelocity.z, collidable.angularVelocity.y,
            collidable.angularVelocity.z, 0, -collidable.angularVelocity.x,
            -collidable.angularVelocity.y, collidable.angularVelocity.x, 0
            );

        angVel *= Time.deltaTime;

        float3x3 rotation = math.mul(angVel, new float3x3(collidable.transform.rotation));
        rotation += new float3x3(collidable.transform.rotation);
        collidable.transform.rotation = OrthonormalizeOrientation(rotation);

        //collidable.invWorldIT = collidable.invBodyIT;

        Vector3 invWorldInertiaTensor = new Vector3(collidable.invBodyIT[0][0], collidable.invBodyIT[1][1], collidable.invBodyIT[2][2]);
        invWorldInertiaTensor = math.mul(collidable.transform.rotation, math.mul(math.conjugate(collidable.transform.rotation), invWorldInertiaTensor));
        //collidable.invWorldIT = math.mul(new float3x3(collidable.transform.rotation) , collidable.invBodyIT);
        //collidable.invWorldIT = math.mul(collidable.invWorldIT, math.transpose(collidable.invBodyIT));
        //collidable.invWorldIT = math.mul(collidable.invBodyIT, math.mul(new float3x3(collidable.transform.rotation), math.transpose(collidable.invBodyIT)));
        collidable.invWorldIT = new float3x3(
            invWorldInertiaTensor.x, 0, 0,
            0, invWorldInertiaTensor.y, 0,
            0, 0, invWorldInertiaTensor.z
            );

        collidable.torque = Vector3.zero;
        collidable.force = Vector3.zero;
    }

    private Quaternion OrthonormalizeOrientation(float3x3 rotation)
    {
        Vector3 X = new Vector3(rotation[0][0], rotation[0][1], rotation[0][2]);
        Vector3 Y = new Vector3(rotation[1][0], rotation[1][1], rotation[1][2]);
        Vector3 Z;

        Z = Vector3.Cross(X, Y);
        Y = Vector3.Cross(Z, X);
        X.Normalize();
        Y.Normalize();
        Z.Normalize();

        float3x3 orientation = new float3x3(
            X.x, Y.x, Z.x,
            X.y, Y.y, Z.y,
            X.z, Y.z, Z.z
            );

        return quaternion(orientation);
    }

    void ShapeCollisionCheck(Collidable a, Collidable b, ref List<CollisionPacket> collisions)
    {
        for (int i = 0; i < a.shapes.Count(); i++)
        {
            for(int j = 0; j < b.shapes.Count(); j++)
            {
                CollisionPacket collision = GJK(a.shapes[i], b.shapes[j], a.transform, b.transform);
                collision.objectA = a;
                collision.objectB = b;

                if (collision.depth > 0)
                    collisions.Add(collision);
            }
        }
    }

    private void Resolution(CollisionPacket collision)
    {
        if (collision.depth < 0) return;
        
        //Debuging
        testingOrbs[0].transform.position = collision.worldContact;

        //Calculate R for each shape
        Vector3 rA = collision.worldContact - collision.objectA.transform.position;
        Vector3 rB = collision.worldContact - collision.objectB.transform.position;
        
        float totalInverseMass = (collision.objectA.invMass + collision.objectB.invMass);
        
        
        // Relative Vel = Liner Vel and Angular Vel at point of Collision
        Vector3 relativeVelocity =
              (collision.objectA.velocity + Vector3.Cross(collision.objectA.angularVelocity,rA))
            - (collision.objectB.velocity + Vector3.Cross(collision.objectB.angularVelocity,rB));
        
        //Calculate Average Elasticity
        float elasticCoef = collision.objectA.elasticCoef + collision.objectB.elasticCoef;
        elasticCoef /= 2;
        
        
        Vector3 aDenomComponentVector = math.mul(collision.objectA.invWorldIT, Vector3.Cross(rA, collision.normal));
        Vector3 bDenomComponentVector = math.mul(collision.objectB.invWorldIT, Vector3.Cross(rB, collision.normal));
        
        float aDenomComponentFloat = Vector3.Dot(Vector3.Cross(aDenomComponentVector, rA),collision.normal);
        float bDenomComponentFloat = Vector3.Dot(Vector3.Cross(bDenomComponentVector, rB),collision.normal);
        
        float denom = aDenomComponentFloat + bDenomComponentFloat;
        
        float j = -(1 + elasticCoef) * Vector3.Dot(relativeVelocity, collision.normal) /
            (totalInverseMass + denom);
         
        if (j < 0) return;
        
        //Add depenertration to collidable, ensures too much depenertration per frame doesnt occure
        AddDepen(collision.normal * (collision.depth) * collision.objectA.invMass / totalInverseMass, ref collision.objectA.netDepen);
        AddDepen(-collision.normal * (collision.depth) * collision.objectB.invMass / totalInverseMass, ref collision.objectB.netDepen);
        
        
        Vector3 linearRestitution = j * collision.normal;
        
        collision.objectA.velocity += linearRestitution * collision.objectA.invMass;
        collision.objectB.velocity -= linearRestitution * collision.objectB.invMass;
        
        Vector3 angularRestitutionA = Vector3.Cross(rA, linearRestitution);
        Vector3 angularRestitutionB = Vector3.Cross(rB, linearRestitution);
        
        collision.objectA.angularMomentum += angularRestitutionA;
        collision.objectB.angularMomentum -= angularRestitutionB;

        //Yes I think you're correct that these shuold be as they are (one plus, one minus) -Finn
    }

    void AddDepen(Vector3 newDepen, ref Vector3 currDepen)
    {
        if (newDepen.sqrMagnitude <= 0.0000001f)
        {
            return;
        }
        if (Vector3.Dot(newDepen, currDepen) <= 0.0000001f)
        {
            currDepen = newDepen + currDepen;
        }
        else
        {
            Vector3 normalNet = Vector3.Normalize(currDepen);
            float amountAlreadyDepened = Vector3.Dot(normalNet, newDepen);
            Vector3 changeInNet = currDepen - amountAlreadyDepened * normalNet;

            if (Vector3.Dot(changeInNet, currDepen) < 0)
            {
                currDepen = newDepen;
            }
            else
            {
                currDepen = changeInNet + newDepen;
            }
        }
    }

    CollisionPacket GJK(Shape a, Shape b, Transform aTransform, Transform bTransform)
    {
        CollisionPacket collision = new CollisionPacket();
        Simplex simp = new Simplex(0);
        Vector3[] shapeA, shapeB;
        shapeA = a.ToArray();
        shapeB = b.ToArray();

        for(int i = 0; i < shapeA.Length; i++)
        {
            shapeA[i] = aTransform.TransformPoint(shapeA[i]);
        }

        for (int i = 0; i < shapeB.Length; i++)
        {
            shapeB[i] = bTransform.TransformPoint(shapeB[i]);
        }


        //Collision Check
        Vector3 direction = Vector3.right;
        Vector3 support = CalculateSupport(shapeA, a.radius, shapeB, b.radius, direction);
        GJKEvolution gjking = AddSupportToSimplex(ref simp, support, direction);
        direction = -support;
        int iter = 0;
        while (gjking == GJKEvolution.evolving && iter < 100)
        {
            iter++;
            support = CalculateSupport(shapeA, a.radius, shapeB, b.radius, direction);
            if (AddSupportToSimplex(ref simp, support, direction) == GJKEvolution.evolving)
            {
                gjking = EvolveSimplex(ref simp, ref direction);
            }
            else
            {
                gjking = GJKEvolution.notIntersecting;
            }
        }

        if (gjking == GJKEvolution.intersecting)
        {
            EPA(ref simp, ref collision, shapeA, a.radius, aTransform, shapeB, b.radius, bTransform);
            CalculateCollsionPoint(shapeA, a, shapeB, b, simp.points, collision.normal ,out collision.worldContact);
        }

        return collision;
    }

    GJKEvolution EvolveSimplex(ref Simplex simp, ref Vector3 direction)
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

    void CalculateCollsionPoint(Vector3[]a, Shape aShape, Vector3[]b, Shape bShape, List<Vector3> points, Vector3 normal, out Vector3 contactPoint)
    {
        contactPoint = Vector3.zero;

        Vector3 aMost = SupportFunction(-normal, a, aShape.radius);
        Vector3 bMost = SupportFunction(normal, b, bShape.radius);

        if (a.Length == 1)
        {
            contactPoint = aMost;
            return;
        }
        else if (b.Length == 1)
        {
            contactPoint = bMost;
            return;
        }

        List<Vector3> aFaceVertices = new List<Vector3>();
        List<Vector3> bFaceVertices = new List<Vector3>();
        int vertexCount = 0;

        for(int i = 0; i < a.Length; i++) 
        {
            Vector3 currentVert = a[i] + aShape.radius * -normal;
            if (Vector3.Dot(currentVert - aMost, -normal) > -0.01f)
            {
                aFaceVertices.Add(currentVert);
                vertexCount++;
            }
        }


        for (int i = 0; i < b.Length; i++)
        {
            Vector3 currentVert = b[i] + bShape.radius * normal;
            if (Vector3.Dot(currentVert - bMost, normal) > -0.01f)
            {
                bFaceVertices.Add(currentVert);
                vertexCount++;
            }
        }

        if (aFaceVertices.Count == 1) {contactPoint = aMost; return; }
        if (bFaceVertices.Count == 1) {contactPoint = bMost; return; }

        Vector3 colUp = (bFaceVertices[0] - bFaceVertices[1]).normalized;
        Vector3 colRight = cross(colUp, normal);

        Vector2[] a2D = new Vector2[aFaceVertices.Count];
        Vector2[] b2D = new Vector2[bFaceVertices.Count];


        for (int i = 0; i < a2D.Length; i++)
        {
            float x = Vector3.Dot(aFaceVertices[i], colRight);
            float y = Vector3.Dot(aFaceVertices[i], colUp);

            a2D[i] = new Vector2(x, y);
        }

        for (int i = 0; i < b2D.Length; i++)
        {
            float x = Vector3.Dot(bFaceVertices[i], colRight);
            float y = Vector3.Dot(bFaceVertices[i], colUp);

            b2D[i] = new Vector2(x, y);
        }
        
        for(int i = 0; i < a2D.Length; i++)
        {
            for(int j = 0; j < b2D.Length; j++)
            {
                
            }
        }




    }

    private void AddFaceVert(Vector3 normal, int originIndex, int currIndex, Shape shape, ref List<int>verts)
    {
        if(Vector3.Dot(normal, shape.verticies[currIndex].vert - shape.verticies[originIndex].vert) > -0.01f)
        {
            if (!verts.Contains(currIndex))
            {
                verts.Add(currIndex);
                for(int i = 0; i < shape.verticies[currIndex].edges.Length; i++)
                {
                    AddFaceVert(normal, originIndex, i, shape, ref verts);
                }
            }
        }
    }


    GJKEvolution AddSupportToSimplex(ref Simplex simp, Vector3 vert, Vector3 dir)
    {
        simp.points.Add(vert);
        return Vector3.Dot(dir, vert) >= 0 ? GJKEvolution.evolving : GJKEvolution.notIntersecting;
    }

    Vector3 CalculateSupport(Vector3[] a, float aRadius, Vector3[] b, float bRadius, Vector3 dir)
    {

        return SupportFunction(dir, a, aRadius) - SupportFunction(-dir, b, bRadius);
    }

    Vector3 SupportFunction(Vector3 dir, Vector3[] vecs, float radius)
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
        return vecs[index] + dir.normalized * radius;
    }

    void EPA(ref Simplex simp, ref CollisionPacket collision, Vector3[] shapeA, float aRadius, Transform a, Vector3[] shapeB, float bRadius, Transform b)
    {
        List<Vector3> polytope = new List<Vector3>(simp.points);
        List<int> faces = new List<int>
            {
                0, 1, 2,
                0, 3, 1,
                0, 2, 3,
                1, 3, 2
            };

        int minFace = 0;
        List<Vector4> normals = GetFaceNormals(polytope, faces, out minFace);

        Vector3 minNormal = Vector3.up;
        float minDistance = float.MaxValue;
        while (minDistance == float.MaxValue)
        {
            minNormal = new Vector3(normals[minFace].x, normals[minFace].y, normals[minFace].z);
            minDistance = normals[minFace].w;

            Vector3 support = CalculateSupport(shapeA, aRadius, shapeB, bRadius, minNormal);
            float sDistance = Vector3.Dot(minNormal, support);

            if (Mathf.Abs(sDistance - minDistance) > 0.001f)
            {
                minDistance = float.MaxValue;
                List<Tuple<int, int>> uniqueEdges = new List<Tuple<int, int>>();
                for (int i = 0; i < normals.Count(); i++)
                {
                    if (Vector3.Dot(normals[i], support) > Vector3.Dot(normals[i], polytope[faces[i * 3]]))
                    {
                        int f = i * 3;
                        AddIfUniqueEdge(ref uniqueEdges, faces, f + 0, f + 1);
                        AddIfUniqueEdge(ref uniqueEdges, faces, f + 1, f + 2);
                        AddIfUniqueEdge(ref uniqueEdges, faces, f + 2, f + 0);

                        faces[f + 2] = faces[faces.Count() - 1]; faces.RemoveAt(faces.Count() - 1);
                        faces[f + 1] = faces[faces.Count() - 1]; faces.RemoveAt(faces.Count() - 1);
                        faces[f + 0] = faces[faces.Count() - 1]; faces.RemoveAt(faces.Count() - 1);

                        normals[i] = normals[normals.Count() - 1];
                        normals.RemoveAt(normals.Count() - 1);
                        i--;
                    }
                }

                List<int> newFaces = new List<int>();
                foreach (Tuple<int, int> edge in uniqueEdges)
                {
                    newFaces.Add(edge.Item1);
                    newFaces.Add(edge.Item2);
                    newFaces.Add(polytope.Count());
                }

                polytope.Add(support);

                int newMinFace = 0;
                List<Vector4> newNormals = GetFaceNormals(polytope, newFaces, out newMinFace);
                float oldMinDistance = float.MaxValue;
                for (int i = 0; i < normals.Count(); i++)
                {
                    if (normals[i].w < oldMinDistance)
                    {
                        oldMinDistance = normals[i].w;
                        minFace = i;
                    }
                }

                float newMinDistance = newNormals[newMinFace].w;

                if (newMinDistance < oldMinDistance)
                {
                    minFace = newMinFace + normals.Count();
                }


                foreach (int edge in newFaces)
                {
                    faces.Add(edge);
                }
                foreach (Vector4 normal in newNormals)
                {
                    normals.Add(normal);
                }
            }
        }

        collision.normal = -Vector3.Normalize(new Vector3(minNormal.x, minNormal.y, minNormal.z) * Vector3.Dot(b.position - a.position, minNormal));
        collision.depth = minDistance;
    }

    List<Vector4> GetFaceNormals(List<Vector3> polytope, List<int> faces, out int face)
    {
        List<Vector4> normals = new List<Vector4>();

        int minTriangle = 0;
        float minDistance = float.MaxValue;
        for(int i = 0; i < faces.Count(); i+=3)
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

