let slideIndex = 0;
const slides = document.querySelectorAll(".carousel-image");

// Function to move slides
function moveSlides(n) {
    slideIndex += n;

    if (slideIndex < 0) {
        slideIndex = slides.length - 1;
    } else if (slideIndex >= slides.length) {
        slideIndex = 0;
    }

    updateCarousel();
}

// Function to update carousel position
function updateCarousel() {
    const carousel = document.querySelector(".carousel");
    const imageWidth = slides[0].clientWidth;
    const offset = -(imageWidth * slideIndex);

    carousel.style.transform = `translateX(${offset}px)`;
}

// Auto-slide every 5 seconds
setInterval(() => {
    moveSlides(1);
}, 5000);
