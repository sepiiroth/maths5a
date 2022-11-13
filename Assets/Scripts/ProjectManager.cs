using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class ProjectManager : MonoBehaviour
{
    private static ProjectManager _singleton;

    public static ProjectManager Instance()
    {
        return _singleton;
    }

    private List<GameObject> pointsList;
    private List<Arete> A;
    private List<Triangle> T;

    [SerializeField]
    private GameObject linePrefab;

    // Start is called before the first frame update
    void Start()
    {
        _singleton = this;
        pointsList = new List<GameObject>();
        A = new List<Arete>();
        T = new List<Triangle>();
    }

    // Update is called once per frame
    void Update()
    {
        //Envelope Convex
        if (Input.GetKeyDown(KeyCode.A))
        {
            List<GameObject> temp = getConvexEnvelopJarvis(pointsList);

            var newLine = Instantiate(linePrefab, Vector3.zero, Quaternion.Euler(0, 0, 0));
            LineRenderer lR = newLine.GetComponent<LineRenderer>();
            lR.positionCount = temp.Count;
            int index = 0;
            foreach (var x in temp)
            {
               
                lR.SetPosition(index, x.transform.position + new Vector3(0,0,-0.1f)) ;
                Debug.Log(x.transform.position);
                index++;
            }
        }
        
        //Triangulation
        if (Input.GetKeyDown(KeyCode.Z))
        {
            IncrementalTriangulation();
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            Flipping();
        }
    }

    public void AddPointToList(GameObject point)
    {
        pointsList.Add(point);
    }
    
    /*********** Enveloppe Convex *************/

    public List<GameObject> getConvexEnvelopJarvis(List<GameObject> points)
    {
        List<GameObject> result = new List<GameObject>();
        
        var pointMin = points[0];
        var xMin = pointMin.transform.position.x;

        
        //Recherche du point le plus à gauche
        for (int i = 1; i < points.Count; i++)
        {
            var xCurrent = points[i].transform.position.x;
            if (xCurrent < xMin)
            {
                pointMin = points[i];
                xMin = xCurrent;
            }
            if (Math.Abs(xCurrent - xMin) < 0.00001f)
            {
                if (points[i].transform.position.y < pointMin.transform.position.y)
                {
                    pointMin = points[i];
                    xMin = xCurrent;
                }
            }
        }
        
        
        
        result.Add(pointMin);
        //
        var currentPoint = pointMin;
        do
        {
            Vector3 lastPoint = new Vector3(currentPoint.transform.position.x, currentPoint.transform.position.y - 1, 0);
            if (result.Count > 1)
            {
                lastPoint = result[result.Count-2].transform.position;
            }

            var minAngle = GetMinAngle(currentPoint.transform.position,lastPoint , points);
            result.Add(minAngle);
            currentPoint = minAngle;
        } while (currentPoint != result[0]);
        
        return result;
    }

    GameObject GetMinAngle(Vector3 currentPoint, Vector3 lastPoint, List<GameObject> points)
    {
        var minAngle = 361f;
        GameObject minPoint = null;
        for (int i = 0; i < points.Count; i++)
        {
            if (points[i].transform.position != currentPoint && points[i].transform.position != lastPoint)
            {
                var angleI = GetAngle(points[i].transform.position, currentPoint, lastPoint);
                if (angleI < minAngle)
                {
                    minAngle = angleI;
                    minPoint = points[i];
                }
            }
        }

        return minPoint;
    }
    
    float GetAngle(Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 u = new Vector3(a.x - b.x, a.y - b.y, a.z - b.z);
        Vector3 v = new Vector3(c.x - b.x, c.y - b.y, c.z - b.z);


        float uNorm = Mathf.Sqrt(Mathf.Pow(u.x, 2) + Mathf.Pow(u.y, 2));
        float vNorm = Mathf.Sqrt(Mathf.Pow(v.x, 2) + Mathf.Pow(v.y, 2));

        
        if(uNorm == 0 | vNorm == 0)
        {
            return 0;
        }

        float dotUV = (u.x * v.x) + (u.y * v.y);
        
        var angle = Mathf.Acos(dotUV/(uNorm * vNorm)) * 180 / Mathf.PI;

        var perp = Vector3.Cross(v, u);
        
        //Test pour connaitre le sens de l'angle
        var temp = -v.x * u.y + v.y * u.x;
        
        if (temp < 0)
        {
            angle = 360 - angle;
        }

        return angle;
        //return Vector3.Angle(u, v);
    }
    
    /************************/
    
    /************ Trinagulation Incrémentale *************/
    void IncrementalTriangulation()
    {
        List<GameObject> pointSorted = pointsList.OrderBy(x => x.transform.position.x).ToList();
        List<GameObject> envelop = new List<GameObject>();
        List<GameObject> polygon = new List<GameObject>();

        var tempList = pointSorted.GetRange(0, 3).OrderBy(x => x.transform.position.y).ToList();
        
        Arete a1 = CreateLine(tempList[0].transform.position, tempList[1].transform.position);
        Arete a2 = CreateLine(tempList[1].transform.position, tempList[2].transform.position);

        Arete a3 = CreateLine(tempList[0].transform.position, pointSorted[3].transform.position);;
        Arete a4 = CreateLine(tempList[1].transform.position, pointSorted[3].transform.position);;
        Arete a5 = CreateLine(tempList[2].transform.position, pointSorted[3].transform.position);;

        Triangle t1 = new Triangle(a1, a3, a4);
        Triangle t2 = new Triangle(a2, a5, a4);
        
        T.Add(t1);
        T.Add(t2);
        
        polygon.Add(pointSorted[0]);
        polygon.Add(pointSorted[1]);
        polygon.Add(pointSorted[2]);
        polygon.Add(pointSorted[3]);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        pointSorted.RemoveAt(0);
        
        envelop = getConvexEnvelopJarvis(polygon);

        while (pointSorted.Count > 0)
        {
            envelop = getConvexEnvelopJarvis(polygon);
            for (int i = 0; i < envelop.Count; i++)
            {
                var left = isLeft(envelop[i].transform.position, envelop[(i+1) % envelop.Count].transform.position,
                    pointSorted[0].transform.position);
                if (left)
                {
                    Arete areteA = CreateLine(envelop[i].transform.position, pointSorted[0].transform.position);
                    Arete areteB = CreateLine(envelop[(i+1)%envelop.Count].transform.position, pointSorted[0].transform.position);
                    Arete areteC = GetAreteFromList(envelop[i].transform.position,
                        envelop[(i + 1) % envelop.Count].transform.position);
                    Triangle t = new Triangle(areteA, areteB, areteC);
                    T.Add(t);
                }
            }
            polygon.Add(pointSorted[0]);
            pointSorted.RemoveAt(0);
        }
        
    }

    Arete GetAreteFromList(Vector3 x, Vector3 y)
    {
        Arete temp = new Arete(x, y);
        for (int i = 0; i < A.Count; i++)
        {
            if (temp.Equals(A[i]))
            {
                return A[i];
            }
        }
        
        return null;
    }

    Arete CreateLine(Vector3 positionA, Vector3 positionB)
    {
        Arete temp = new Arete(positionA, positionB);
        if (!A.Exists(x => x.Equals(temp)))
        {
            var lineTemp = Instantiate(linePrefab, Vector3.zero, Quaternion.Euler(0, 0, 0));
            LineRenderer lrTemp = lineTemp.GetComponent<LineRenderer>();
            lrTemp.positionCount = 2;
            lrTemp.SetPosition(0, positionA);
            lrTemp.SetPosition(1, positionB);
        
        
            temp.SetLineRenderer(lrTemp);
            A.Add(temp);
        }
        else
        {
            temp = A.Find(x => x.Equals(temp));
        }
        
        return temp;
    }
    
    
    //a = Point de la ligne
    //b = Point de la ligne
    //c = Point à tester
    public bool isLeft(Vector3 a, Vector3 b, Vector3 c){
        return ((b.x - a.x)*(c.y - a.y) - (b.y - a.y)*(c.x - a.x)) > 0;
    }
    
    //Flipping
    void Flipping()
    {
        List<Arete> Ac = A.ToList();
        while (Ac.Count > 0)
        {
            var a = Ac[0];
            Ac.RemoveAt(0);
            Vector3 S1 = a.GetPointA();
            Vector3 S2 = a.GetPointB();
            
            List<Triangle> temp = GetTriangleFromArete(a);
            //Debug.Log("-----------------------------");

            if (temp.Count < 2)
            {
                continue;
            }
            
            

            Triangle T1 = temp[0];
            Triangle T2 = temp[1];
            
            List<Vector3> pointTA = T1.GetSommet();
            List<Vector3> pointTB = T2.GetSommet();

            pointTA.Remove(a.GetPointA());
            pointTA.Remove(a.GetPointB());
        
            pointTB.Remove(a.GetPointA());
            pointTB.Remove(a.GetPointB());

            Vector3 S3 = pointTA[0];
            Vector3 S4 = pointTB[0];
            

            List<Arete> areteT1 = T1.GetAllArete();
            areteT1.Remove(a);
            List<Arete> areteT2 = T2.GetAllArete();
            areteT2.Remove(a);
            
            Arete A1 = areteT1[0];
            Arete A2 = areteT1[1];
            Arete A3 = areteT2[0];
            Arete A4 = areteT2[1];

            if (!CheckDelaunayCriteria(a, S3, S4))
            {
                a.SetPointA(S3);
                a.SetPointB(S4);

                List<Arete> A1234 = new List<Arete>() {A1, A2, A3, A4};

                List<Arete> newT1 = A1234.Where(x => x.GetPointA().Equals(S1) | x.GetPointB().Equals(S1)).ToList();
                List<Arete> newT2 = A1234.Where(x => x.GetPointA().Equals(S2) | x.GetPointB().Equals(S2)).ToList();

                T1 = new Triangle(a, newT1[0], newT1[1]);
                T2 = new Triangle(a, newT2[0], newT2[1]);
                
                Ac.Add(A1);
                Ac.Add(A2);
                Ac.Add(A3);
                Ac.Add(A4);
            }
            
        }
    }

    public bool CheckDelaunayCriteria(Arete a, Vector3 DA, Vector3 DB)
    {
        //Autre methode mais ne fonctionne pas
        /*List<Vector3> pointsT1 = new List<Vector3>(){a.GetPointA(), a.GetPointB(), DA}.OrderBy(x => x.x).ToList();
        List<Vector3> pointsT2 = new List<Vector3>(){a.GetPointA(), a.GetPointB(), DB}.OrderBy(x => x.x).ToList();

        var leftA = pointsT1[0];
        var middleA = pointsT1[1].y < pointsT1[2].y ? pointsT1[1] : pointsT1[2];
        var rightA = pointsT1[1].y < pointsT1[2].y ? pointsT1[2] : pointsT1[1];
        
        var leftB = pointsT2[0];
        var middleB = pointsT2[1].y < pointsT2[2].y ? pointsT2[1] : pointsT2[2];
        var rightB = pointsT2[1].y < pointsT2[2].y ? pointsT2[2] : pointsT2[1];
        
        //Debug.Log($"TA = {MatDet(leftA, middleA, rightA, DB)} - TB = {MatDet(leftB, middleB, rightB, DA)}");
        return MatDet(leftA, middleA, rightA, DB) > 0 & MatDet(leftB, middleB, rightB, DA) > 0;*/

        float angleT1 = GetAngle(a.GetPointA(),DA, a.GetPointB());
        float angleT2 = GetAngle(a.GetPointA(),DB, a.GetPointB());

        angleT1 = angleT1 > 180 ? 360 - angleT1 : angleT1;
        angleT2 = angleT2 > 180 ? 360 - angleT2 : angleT2;

        return angleT1 + angleT2 <= 180;
    }

    public float MatDet(Vector3 A, Vector3 B, Vector3 C, Vector3 D)
    {
        float a = A.x - D.x;
        float b = A.y - D.y;
        float c = (Mathf.Pow(A.x, 2) - Mathf.Pow(D.x, 2)) + (Mathf.Pow(A.y, 2) - Mathf.Pow(D.y, 2));
        
        float d = B.x - D.x;
        float e = B.y - D.y;
        float f = (Mathf.Pow(A.x, 2) - Mathf.Pow(D.x, 2)) + (Mathf.Pow(A.y, 2) - Mathf.Pow(D.y, 2));
        
        float g = C.x - D.x;
        float h = C.y - D.y;
        float i = (Mathf.Pow(A.x, 2) - Mathf.Pow(D.x, 2)) + (Mathf.Pow(A.y, 2) - Mathf.Pow(D.y, 2));

        return (a * e * i) - (a * f * h) + (b * f * g) - (b * d * i) + (c * d * h) - (c * e * g);
    }

    public List<Triangle> GetTriangleFromArete(Arete A)
    {
        List<Triangle> temp = new List<Triangle>();
        foreach (var triangle in T)
        {
            if (triangle.ContainsArete(A))
            {
                temp.Add(triangle);
                /*foreach (var s in triangle.GetSommet())
                {
                    Debug.Log(s);
                }*/
            }
        }

        
        return temp;
    }

}


