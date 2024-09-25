let slideIndex = 0;
showSlides();

function showSlides() {
  let i;
  let slides = document.getElementsByClassName("mySlides");
  
  // Hide all slides
  for (i = 0; i < slides.length; i++) {
    slides[i].style.display = "none";
  }

  // Increment slideIndex and wrap around
  slideIndex++;
  if (slideIndex > slides.length) {
    slideIndex = 1; // Reset to 1 if exceeds
  }

  // Show the current slide
  slides[slideIndex - 1].style.display = "block"; // Display the correct slide
  setTimeout(showSlides, 2000); // Change image every 2 seconds
}
