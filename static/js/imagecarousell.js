let slideIndex = [1, 1];  // Array for multiple carousels
let slideId = ["mySlides1", "mySlides2"]; // Array of slide class names

showSlides(1, 0);  // Show the first carousel (0 is index for carousel 1)
showSlides(2, 0);  // Show the second carousel (1 is index for carousel 2)

// Controls for next/prev
function plusSlides(n, carousel) {
    showSlides(slideIndex[carousel] += n, carousel);
}

// Function to show slides for a specific carousel
function showSlides(n, carousel) {
    let i;
    let slides = document.getElementsByClassName(slideId[carousel]);  // Get slides based on carousel index
    if (n > slides.length) { slideIndex[carousel] = 1; }
    if (n < 1) { slideIndex[carousel] = slides.length; }
    for (i = 0; i < slides.length; i++) {
        slides[i].style.display = "none";  // Hide all slides in the carousel
    }
    slides[slideIndex[carousel] - 1].style.display = "block";  // Show the current slide
}