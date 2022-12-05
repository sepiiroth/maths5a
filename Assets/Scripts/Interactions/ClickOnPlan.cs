using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClickOnPlan : MonoBehaviour
{
    private Camera mainCamera;
    private Vector3 mousePosition;

    public GameObject prefabTemporaire; //A supprimer au bout d'un moment


    private GameObject tempObj;
    // Start is called before the first frame update
    void Start()
    {
        mainCamera = Camera.main;
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {

            int layerMaskBack = 1 << 6;
            int layerMask = 1 << 8;
            layerMask = ~layerMask;
            RaycastHit hit;

            mousePosition = Input.mousePosition;

            Vector3 newV = mainCamera.ScreenToWorldPoint(new Vector3(mousePosition.x, mousePosition.y, 10f));
            
            if (Physics.Raycast(mainCamera.transform.position, Vector3.forward, out hit, Mathf.Infinity, layerMaskBack))
            {

                tempObj = (GameObject) Instantiate(prefabTemporaire, newV, Quaternion.Euler(0, 0, 0) );
                tempObj.tag = "Point";
                tempObj.layer = LayerMask.NameToLayer("Points");
                
            }
            
        }

        if (Input.GetMouseButtonDown(1))
        {
            int layerMaskBack = 1 << 7;
            RaycastHit hit;

            mousePosition = Input.mousePosition;

            Vector3 newV = mainCamera.ScreenToWorldPoint(new Vector3(mousePosition.x, mousePosition.y, 10f));

            Vector3 dir = (newV - mainCamera.transform.position).normalized;
            
            if (Physics.Raycast(mainCamera.transform.position, dir, out hit, Mathf.Infinity, layerMaskBack))
            {
                //Debug.Log(hit.collider.gameObject);
                GameObject temp = (GameObject) hit.collider.gameObject;
                ProjectManager.Instance().RemovePointToList(temp);
                Destroy(temp);
            }
        }

        if (Input.GetMouseButton(0))
        {
            if(Input.GetAxis("Mouse ScrollWheel") != 0f) {
                GameObject lastPoint = tempObj;
                lastPoint.transform.Translate(Vector3.forward * Input.GetAxis("Mouse ScrollWheel"));
            }
        }

        if (Input.GetMouseButtonUp(0))
        {
            ProjectManager.Instance().AddPointToList(tempObj);
        }
        
    }
}
