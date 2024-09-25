let slideIndex1 = 1;
let slideIndex2 = 1;
showSlides(slideIndex1, 1);
showSlides(slideIndex2, 2);

function plusSlides(n, carousel) {
    if (carousel === 1) {
        showSlides(slideIndex1 += n, 1);
    } else {
        showSlides(slideIndex2 += n, 2);
    }
}

function currentSlide(n, carousel) {
    if (carousel === 1) {
        showSlides(slideIndex1 = n, 1);
    } else {
        showSlides(slideIndex2 = n, 2);
    }
}

function showSlides(n, carousel) {
    let i;
    let slides = document.getElementsByClassName("mySlides");
    let dots = document.getElementsByClassName("dot");
    let totalSlides = carousel === 1 ? 3 : 3; // Total slides per carousel
    if (n > totalSlides) { 
        if (carousel === 1) slideIndex1 = 1; 
        else slideIndex2 = 1; 
    }
    if (n < 1) { 
        if (carousel === 1) slideIndex1 = totalSlides; 
        else slideIndex2 = totalSlides; 
    }
    for (i = 0; i < slides.length; i++) {
        if (i < totalSlides && (carousel === 1 ? i < 3 : i >= 3)) {
            slides[i].style.display = "none";  // Hide all slides
        }
    }
    for (i = 0; i < dots.length; i++) {
        dots[i].className = dots[i].className.replace(" active", "");  // Remove 'active' class from dots
    }
    if (carousel === 1) {
        slides[slideIndex1 - 1].style.display = "block";  // Display the current slide for carousel 1
        dots[slideIndex1 - 1].className += " active";  // Set the current dot to active for carousel 1
    } else {
        slides[slideIndex2 - 1 + 3].style.display = "block";  // Display the current slide for carousel 2
        dots[slideIndex2 - 1 + 3].className += " active";  // Set the current dot to active for carousel 2
    }
}