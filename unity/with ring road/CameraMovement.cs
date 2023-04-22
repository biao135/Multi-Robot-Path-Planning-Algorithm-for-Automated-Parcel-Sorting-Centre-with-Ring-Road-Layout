using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    [SerializeField]
    private Camera cam;
    [SerializeField]
    private float zoomStep = 1;
    [SerializeField]
    private float minCamSize = 3;
    [SerializeField]
    private float maxCamSize = 20;

    private Vector3 dragOrigin;
    private Vector3 difference;
    private float newSize;

    // Update is called once per frame
    void Update()
    {
        if (Input.GetMouseButtonDown(0)){
            dragOrigin = cam.ScreenToWorldPoint(Input.mousePosition);
        }

        if(Input.GetMouseButton(0)){
            difference = dragOrigin - cam.ScreenToWorldPoint(Input.mousePosition);
            cam.transform.position += difference;

        }

        if (Input.GetAxis("Mouse ScrollWheel") > 0f ){
            newSize = cam.orthographicSize - zoomStep;
            cam.orthographicSize = Mathf.Clamp(newSize, minCamSize, maxCamSize);
        }
        
        if (Input.GetAxis("Mouse ScrollWheel") < 0f ){
            newSize = cam.orthographicSize + zoomStep;
            cam.orthographicSize = Mathf.Clamp(newSize, minCamSize, maxCamSize);
        }
    }
}
