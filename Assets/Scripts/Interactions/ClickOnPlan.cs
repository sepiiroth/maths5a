using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ClickOnPlan : MonoBehaviour
{
    private Camera mainCamera;
    private Vector3 mousePosition;

    public GameObject prefabTemporaire; //A supprimer au bout d'un moment
    
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

                GameObject temp = (GameObject) Instantiate(prefabTemporaire, newV, Quaternion.Euler(0, 0, 0) );
                temp.tag = "Point";
                temp.layer = LayerMask.NameToLayer("Points");
            }
            
        }
    }
}
