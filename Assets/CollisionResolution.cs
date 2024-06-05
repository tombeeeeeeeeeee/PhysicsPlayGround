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


public class CollisionResolution : MonoBehaviour
{
    public GameObject[] testingOrbs;

    const float COLLISION_FACE_THRESHOLD = -0.025f;
    public Vector3 gravity = new Vector3(0, -9.8f, 0);

    public Collidable[] collidables;


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

        for(int iter = 0; iter < 10; iter++)
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

        collidable.velocity *= 1 - 0.0001f * collidable.drag;
        collidable.velocity += Time.deltaTime * collidable.invMass * collidable.force;
        collidable.angularMomentum *= 1 - 0.01f * collidable.angularDrag;
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

        collidable.invWorldIT = math.mul(new float3x3(collidable.transform.rotation) , collidable.invBodyIT);
        collidable.invWorldIT = math.mul(collidable.invWorldIT, math.transpose(collidable.invBodyIT));


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
         
        if (j <= 0) return;
        
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
        List<Vector3> simp = new List<Vector3>();
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
            CalculateCollsionPoint(shapeA, a, shapeB, b, collision.normal ,out collision.worldContact);
        }

        return collision;
    }

    GJKEvolution EvolveSimplex(ref List<Vector3> simp, ref Vector3 direction)
    {
        switch (simp.Count)
        {
            case 2:

                // line ab is the line formed by the first two vertices
                Vector3 ab = simp[1] - simp[0];
                // line a0 is the line from the first vertex to the origin
                Vector3 dO = simp[0] * -1;

                if (Vector3.Dot(ab, dO) > 0)
                {
                    // use the triple-cross-product to calculate shapeA direction perpendicular
                    // to line ab in the direction of the origin
                    Vector3 temp = Vector3.Cross(ab, dO);
                    direction = Vector3.Cross(temp, ab);
                }
                else
                {
                    simp.RemoveAt(1);
                    direction = dO;
                }

                return GJKEvolution.evolving;

            case 3:
                Vector3 ac = simp[2] - simp[0];
                ab = simp[1] - simp[0];
                dO = simp[0] * -1;

                direction = Vector3.Cross(ac, ab);

                if (Vector3.Dot(ac, dO) <= 0) direction *= -1;

                break;
            case 4:
                // calculate the three edges of interest
                Vector3 da = simp[0] - simp[3];
                Vector3 db = simp[1] - simp[3];
                Vector3 dc = simp[2] - simp[3];

                // and the direction to the origin
                dO = simp[3] * -1;

                // check triangles a-b-d, b-c-d, and c-a-d
                Vector3 abdNorm = Vector3.Cross(da, db);
                Vector3 bcdNorm = Vector3.Cross(db, dc);
                Vector3 cadNorm = Vector3.Cross(dc, da);

                if (Vector3.Dot(bcdNorm, dO) > 0)
                {
                    simp.RemoveAt(0);
                    direction = bcdNorm;
                }
                else if (Vector3.Dot(abdNorm, dO) > 0)
                {
                    simp.RemoveAt(2);
                    direction = abdNorm;
                }
                else if (Vector3.Dot(cadNorm, dO) > 0)
                {
                    simp.RemoveAt(1);
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

    void CalculateCollsionPoint(Vector3[] aVerts, Shape aShape, Vector3[] bVerts, Shape bShape, Vector3 normal, out Vector3 contactPoint)
    {
        contactPoint = Vector3.zero;

        int aMostIndex, bMostIndex;

        Vector3 aMost = SupportFunction(-normal, aVerts, aShape.radius, out aMostIndex);
        Vector3 bMost = SupportFunction(normal, bVerts, bShape.radius, out bMostIndex);

        if (aVerts.Length == 1)
        {
            contactPoint = aMost;
            return;
        }
        else if (bVerts.Length == 1)
        {
            contactPoint = bMost;
            return;
        }

        List<int> aFaceVertices = new List<int>();aFaceVertices.Add(aMostIndex);
        List<int> bFaceVertices = new List<int>();bFaceVertices.Add(bMostIndex);

        for(int i = 0; i < aShape.vertices[aMostIndex].edges.Length; i++)
        {
            AddFaceVert(-normal, aMostIndex, aShape.vertices[aMostIndex].edges[i], aVerts, aShape, ref aFaceVertices);
        }

        for (int i = 0; i < bShape.vertices[bMostIndex].edges.Length; i++)
        {
            AddFaceVert(normal, bMostIndex, bShape.vertices[bMostIndex].edges[i], bVerts, bShape, ref bFaceVertices);
        }

        if (aFaceVertices.Count == 1) {contactPoint = aMost; return; }
        if (bFaceVertices.Count == 1) {contactPoint = bMost; return; }

        Vector3 colUp = (bVerts[bFaceVertices[0]] - bVerts[bFaceVertices[1]]).normalized;
        Vector3 colRight = Vector3.Cross(colUp, normal).normalized;
        colUp = Vector3.Cross(normal, colRight).normalized;

        Vector2[] a2D = new Vector2[aFaceVertices.Count];
        Vector2[] b2D = new Vector2[bFaceVertices.Count];


        for (int i = 0; i < a2D.Length; i++)
        {
            float x = Vector3.Dot(aVerts[aFaceVertices[i]], colRight);
            float y = Vector3.Dot(aVerts[aFaceVertices[i]], colUp);
            a2D[i] = new Vector2(x, y);
        }

        for (int i = 0; i < b2D.Length; i++)
        {
            float x = Vector3.Dot(bVerts[bFaceVertices[i]], colRight);
            float y = Vector3.Dot(bVerts[bFaceVertices[i]], colUp);

            b2D[i] = new Vector2(x, y);
        }

        List<Vector2> contactPoints = new List<Vector2>();

        for(int i = 0; i < a2D.Length; i++)
        {
            Vector2 a = a2D[i];
            Vector2 b = a2D[(i + 1) % a2D.Length] - a;

            for(int j = 0; j < b2D.Length; j++)
            {
                Vector2 c = b2D[j];
                Vector2 d = b2D[(j + 1) % b2D.Length] - c;

                float denominator = d.y * b.x - b.y * d.x;
                if(denominator != 0)
                {
                    float numerator = a.y * b.x - b.y * a.x - c.y * b.x + b.y * c.x;
                    float t2 = numerator / denominator;
                    if(t2 >= 0 && t2 <= 1)
                    {
                        float t1 = -1;
                        if(abs(b.x) > abs(b.y))
                        {
                            t1 = c.x + d.x * t2 - a.x;
                            t1 /= b.x;
                        }
                        else if(b.y != 0)
                        {
                            t1 = c.y + d.y * t2 - a.y;
                            t1 /= b.y;
                        }
                        if(t1 >= 0 && t1 <= 1)
                        {
                            Vector2 contact = c + d * t2;
                            contactPoints.Add(contact);
                        }
                    }
                }
            }
        }

        if(contactPoints.Count > 0)
        {
            contactPoint = Vector3.zero;
            foreach(Vector2 vert in contactPoints)
            {
                contactPoint += vert.x * colRight + vert.y * colUp;
            }
            contactPoint /= contactPoints.Count;
            contactPoint += normal * (Vector3.Dot(normal, aMost) + Vector3.Dot(normal, bMost))/2;
        }
        else
        {
            Vector2 aApproxCentre = a2D[0];
            Vector2 bApproxCentre = Vector2.zero;

            for(int i = 0; i < b2D.Length; i++)
            {
                bApproxCentre += b2D[i];
            }
            Vector2 contactPoint2D;
            Vector2 planeNormal = b2D[1] - b2D[0];
            planeNormal = new Vector2(planeNormal.y, -planeNormal.x);
            float planeDisplacement = Vector2.Dot(b2D[1], planeNormal);
            float depthSign = Vector2.Dot(a2D[0], planeNormal) - planeDisplacement;
            for(int i = 1; i < a2D.Length; i++)
            {
                if(sign(depthSign) != sign(Vector2.Dot(a2D[i], planeNormal) - planeDisplacement))
                {
                    contactPoint2D = bApproxCentre / b2D.Length;
                    contactPoint = contactPoint2D.x * colRight + contactPoint2D.y * colUp;
                    contactPoint += normal * (Vector3.Dot(normal, aMost) + Vector3.Dot(normal, bMost)) / 2;
                    return;
                }
                aApproxCentre += a2D[i];
            }

            contactPoint2D = aApproxCentre / a2D.Length;
            contactPoint = contactPoint2D.x * colRight + contactPoint2D.y * colUp;
            contactPoint += normal * (Vector3.Dot(normal, aMost) + Vector3.Dot(normal, bMost)) / 2;
            return;
        }
    }

    private void AddFaceVert(Vector3 normal, int originIndex, int currIndex, Vector3[] worldSpace, Shape shape, ref List<int>verts)
    {
        if(Vector3.Dot(normal, (worldSpace[currIndex] - worldSpace[originIndex])) > COLLISION_FACE_THRESHOLD)
        {
            if (!verts.Contains(currIndex))
            {
                verts.Add(currIndex);
                for(int i = 0; i < shape.vertices[currIndex].edges.Length; i++)
                {
                    AddFaceVert(normal, originIndex, shape.vertices[currIndex].edges[i], worldSpace, shape, ref verts);
                }
            }
        }
    }


    GJKEvolution AddSupportToSimplex(ref List<Vector3> simp, Vector3 vert, Vector3 dir)
    {
        simp.Add(vert);
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

    Vector3 SupportFunction(Vector3 dir, Vector3[] vecs, float radius, out int index)
    {
        float max = float.NegativeInfinity;
        index = 0;
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

    void EPA(ref List<Vector3> simp, ref CollisionPacket collision, Vector3[] shapeA, float aRadius, Transform a, Vector3[] shapeB, float bRadius, Transform b)
    {
        List<Vector3> polytope = new List<Vector3>(simp);
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

