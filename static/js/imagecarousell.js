$(function() {
  'use strict';
  
	$('.carousel .carousel-item[data-src]').each(function() {
		var $this = $(this);

		$this.prepend([
			'<div style="background-image: url(', $this.attr('data-src'), ')"></div>'
		].join(''));
	});
});