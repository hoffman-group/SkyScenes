let slideIndex = 0;
let slides = document.getElementsByClassName("mySlides");

function showSlides() {
  let i;

  // Loop through all the slides and reset their display state
  for (i = 0; i < slides.length; i++) {
    slides[i].style.display = "none";  
  }

  // Increment slide index, reset to 0 if it exceeds the number of slides
  slideIndex++;
  if (slideIndex > slides.length) {
    slideIndex = 1; // Reset to first slide
  }

  // Show the current slide
  slides[slideIndex - 1].style.display = "block";  

  // Repeat the process after 3 seconds
  setTimeout(showSlides, 3000); // Change image every 3 seconds
}

// Start the carousel
showSlides();
