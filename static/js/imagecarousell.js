let slideIndex = [1, 1, 1];  // Array to track current slide for each carousel
let slideId = ["mySlides1", "mySlides2", "mySlides3"];  // Carousel slide class names

showSlides(1, 0);  // Initialize carousel 1
showSlides(2, 1);  // Initialize carousel 2
showSlides(3, 2);  // Initialize carousel 2

// Next/previous controls
function plusSlides(n, carouselIndex) {
    showSlides(slideIndex[carouselIndex] += n, carouselIndex);
}

// Thumbnail controls
function currentSlide(carouselIndex, n) {
    showSlides(slideIndex[carouselIndex] = n, carouselIndex);
}

function showSlides(n, carouselIndex) {
    let i;
    let slides = document.getElementsByClassName(slideId[carouselIndex]);
    let dots = document.getElementsByClassName('dot' + (carouselIndex + 1));

    if (n > slides.length) {
        slideIndex[carouselIndex] = 1;
    }
    if (n < 1) {
        slideIndex[carouselIndex] = slides.length;
    }

    for (i = 0; i < slides.length; i++) {
        slides[i].style.display = "none";
    }

    for (i = 0; i < dots.length; i++) {
        dots[i].className = dots[i].className.replace(" active", "");
    }

    slides[slideIndex[carouselIndex] - 1].style.display = "block";
    dots[slideIndex[carouselIndex] - 1].className += " active";
}