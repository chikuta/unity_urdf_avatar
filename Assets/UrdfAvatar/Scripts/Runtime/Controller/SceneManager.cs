using UnityEngine;


namespace UrdfAvatar
{
    class SceneManager : MonoBehaviour
    {
        private uint slideIndex = 1;
        private uint maxSlideIndex = 14;
        private bool isPushed = false;

        void ChangeSlide(uint index)
        {
            string mateiral_name = string.Format("Materials/{0:D3}", index);
            Material material = Resources.Load(mateiral_name) as Material;
            if (material == null)
            {
                material = Resources.Load("Materials/Black") as Material;
            }
            // set material
            var renderer = GetComponent<Renderer>();
            renderer.material = material;
        }

        void Start()
        {
            ChangeSlide(1);
        }

        void Update()
        {
            if (Input.GetKeyDown(KeyCode.DownArrow))
            {
                slideIndex++;
                if (slideIndex > maxSlideIndex)
                {
                    slideIndex = maxSlideIndex;
                }
                ChangeSlide(slideIndex);
            }
            else if (Input.GetKeyDown(KeyCode.UpArrow))
            {
                if (slideIndex > 0) slideIndex--;
                ChangeSlide(slideIndex);
            }
        }
    }
 }