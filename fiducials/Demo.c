// Copyright (c) 2013 by Wayne C. Gramlich.  All rights reserved.

typedef struct Fiducials__Struct *Fiducials;
typedef struct Neighbor__Struct *Neighbor;
typedef struct Tag__Struct *Tag;

#include "Character.h"
#include "CRC.h"
#include "CV.h"
#include "Double.h"
#include "File.h"
#include "FEC.h"
#include "Float.h"
#include "High_GUI2.h"
#include "Integer.h"
#include "List.h"
#include "Logical.h"
#include "String.h"
#include "Unsigned.h"

typedef Logical Mapping[64];

struct Fiducials__Struct {
    CV_Scalar blue;
    Logical blur;
    CV_Point2D32F_Vector corners_vector;
    CV_Scalar cyan;
    CV_Image debug_image;
    Unsigned debug_index;
    CV_Image edge_image;
    FEC fec;
    CV_Image gray_image;
    CV_Scalar green;
    CV_Point origin;
    CV_Image original_image;
    Logical **mappings;
    CV_Scalar purple;
    CV_Scalar red;
    CV_Point2D32F_Vector references;
    CV_Point2D32F_Vector sample_points;
    CV_Size size_5x5;
    CV_Size size_m1xm1;
    CV_Memory_Storage storage;
    Logical tag_bits[64];	// FIXME: Make this Logical *tag_bits;
    CV_Term_Criteria term_criteria;
};

/// @brief A *Tag_Struct* represents one fiducial tag.
struct Tag__Struct {
    /// @brief The angle from the X axis to the tag bottom edge.
    Float floor_angle;

    /// @brief Absolute X coordinate.
    Float floor_x;

    /// @brief Absolute Y coordinate.
    Float floor_y;

    /// @brief Tag identifier.
    Unsigned id;

    /// @brief List *Neighbor* arcs.
    List neighbors;
};

/// @brief A *Neighbor_Struct* represents arc from an *origin* *Tag* to a
/// *target* *Tag*.
struct Neighbor__Struct {
    /// @brief The distance between the origin and the target.
    Float distance;

    /// @brief Distance between camera center and line connecting both tags.
    Float goodness;

    /// @brief The origin *Tag*.
    Tag origin;

    /// @brief The target *Tag*.
    Tag target;

    /// @brief The angle from the origin bottom edge to the target center.
    Float target_angle;

    /// @brief The angle from the origin bottom edge to the target bottom edge.
    Float target_twist;
};

void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points);
extern CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners);
extern Fiducials Fiducials__create(CV_Image original_image);
extern void Fiducials__image_show(Fiducials fiducials, Logical show);
extern Unsigned Fiducials__process(Fiducials fiducials);
extern void Fiducials__sample_points_helper(
  String label, CV_Point2D32F corner, CV_Point2D32F sample_point);

Integer main(Unsigned arguments_size, String arguments[]) {
    File__format(stdout, "Hello\n");
    if (arguments_size <= 1) {
	File__format(stderr, "Usage: Demo *.tga\n");
    } else {
        String image_file_name = arguments[1];
	CV_Image original_image = (CV_Image)0;
	original_image = CV__tga_read(original_image, image_file_name);
	Fiducials fiducials = Fiducials__create(original_image);
	Fiducials__image_show(fiducials, (Logical)1);
    }
    return 0;
}


// This routinew will show {original_image} on the screen along
// with a primitive debugging interface to showing how the debugging
// is going.

void Fiducials__image_show(Fiducials fiducials, Logical show) {
    // Grab some values out of *fiduicals*:
    CV_Image debug_image = fiducials->debug_image;
    CV_Image gray_image = fiducials->gray_image;
    CV_Image original_image = fiducials->original_image;

    // Create the window we need:
    String window_name = "Example1";
    if (show) {
	CV__named_window(window_name, CV__window_auto_size);
    }

    // Processing *original_image* with different options
    // for each time through the loop:
    Unsigned debug_index = 0;
    Unsigned previous_debug_index = debug_index;
    Logical done = (Logical)0;
    while (!done) {
        // Process {gray_image}; a debug image lands in {debug_image}:
	Fiducials__process(fiducials);

	// Display either *original_image* or *debug_image*:
	if (show) {
	    CV_Image__show(debug_image, window_name);
	}

	// Get a *control_character* from the user:
	Character control_character = '\0';
	if (show) {
	    control_character = (Character)(CV__wait_key(0) & 0xff);
	}

	// Dispatch on *control_character*:
	switch (control_character) {
	  case '\33':
	    //# Exit program:
	    done = (Logical)1;
	    File__format(stderr, "done\n");
	    break;
	  case '+':
	    //# Increment {debug_index}:
	    debug_index += 1;
	    break;
	  case '-':
	    // Decrement {debug_index}:
	    if (debug_index > 0) {
		debug_index -= 1;
	    }
	    break;
	  case '<':
	    // Set {debug_index} to beginning:
	    debug_index = 0;
	    break;
	  case '>':
	    // Set {debug_index} to end:
	    debug_index = 100;
	    break;
	  case 'b':
	    // Toggle image blur:
	    fiducials->blur = !fiducials->blur;
	    File__format(stderr, "blur = %d\n", fiducials->blur);
	    break;
	  default:
	    // Deal with unknown {control_character}:
	    if ((Unsigned)control_character <= 127) {
		File__format(stderr,
		  "Unknown control character %d\n", control_character);
	    }
	    break;
	}

	// Update *debug_index* in *fiducials*:
	fiducials->debug_index = debug_index;

	// Show user *debug_index* if it has changed:
	if (debug_index != previous_debug_index) {
	  File__format(stderr,
	    "****************************debug_index = %d\n", debug_index);
	  previous_debug_index = debug_index;
	}
    }

    // Release storage:
    CV__release_image(original_image);
    if (show) {
	CV__destroy_window(window_name);
    }
}

Fiducials Fiducials__create(CV_Image original_image) {
    // Create *image_size*:
    Unsigned width = CV_Image__width_get(original_image);
    Unsigned height = CV_Image__height_get(original_image);
    CV_Size image_size = CV_Size__create(width, height);
    CV_Memory_Storage storage = CV_Memory_Storage__create(0);

    Integer term_criteria_type =
      CV__term_criteria_iterations | CV__term_criteria_eps;

    // The north/west/south/east mappings must reside in static
    // memory rather than on the stack:

    static Logical north_mapping[64] = {
        //corner1              corner0
	 0,  1,  2,  3,  4,  5,  6,  7,
	 8,  9, 10, 11, 12, 13, 14, 15,
	16, 17, 18, 19, 20, 21, 22, 23,
	24, 25, 26, 27, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59, 60, 61, 62, 63,
	//corner2              corner3
    };

    static Logical west_mapping[64] = {
	//corner1              corner0
	 7, 15, 23, 31, 39, 47, 55, 63,
	 6, 14, 22, 30, 38, 46, 54, 62,
	 5, 13, 21, 29, 37, 45, 53, 61,
	 4, 12, 20, 28, 36, 44, 52, 60,
	 3, 11, 19, 27, 35, 43, 51, 59,
	 2, 10, 18, 26, 34, 42, 50, 58,
	 1,  9, 17, 25, 33, 41, 49, 57,
	 0,  8, 16, 24, 32, 40, 48, 56,
	//corner2              corner3
    };

    static Logical south_mapping[64] = {
	//corner1              corner0
	63, 62, 61, 60, 59, 58, 57, 56,
	55, 54, 53, 52, 51, 50, 49, 48,
	47, 46, 45, 44, 43, 42, 41, 40,
	39, 38, 37, 36, 35, 34, 33, 32,
	31, 30, 29, 28, 27, 26, 25, 24,
	23, 22, 21, 20, 19, 18, 17, 16,
	15, 14, 13, 12, 11, 10,  9,  8,
	 7,  6,  5,  4,  3,  2,  1,  0,
	//corner2              corner3
    };

    static Logical east_mapping[64] = {
	//corner1              corner0
	56, 48, 40, 32, 24, 16,  8,  0,
	57, 49, 41, 33, 25, 17,  9,  1,
	58, 50, 42, 34, 26, 18, 10,  2,
	59, 51, 43, 35, 27, 19, 11,  3,
	60, 52, 44, 36, 28, 20, 12,  4,
	61, 53, 45, 37, 29, 21, 13,  5,
	62, 54, 46, 38, 30, 22, 14,  6,
	63, 55, 47, 39, 31, 23, 15,  7,
	//corner2              corner3
    };

    static Logical *mappings[4] = {
	&north_mapping[0],
	&west_mapping[0],
	&south_mapping[0],
	&east_mapping[0],
    };

    //for (Unsigned index = 0; index < 4; index++) {
    //	File__format(stderr, "mappings[%d]=0x%x\n", index, mappings[index]);
    //}

    // Create and load *fiducials*:
    Fiducials fiducials = Memory__new(Fiducials);
    fiducials->blue = CV_Scalar__rgb(0.0, 0.0, 1.0);
    fiducials->blur = (Logical)1;
    fiducials->corners_vector = CV_Point2D32F_Vector__create(4);
    fiducials->cyan = CV_Scalar__rgb(0.0, 1.0, 1.0);
    fiducials->debug_image = CV_Image__create(image_size, CV__depth_8u, 3);
    fiducials->debug_index = 0;
    fiducials->edge_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->fec = FEC__create(8, 4, 4);
    fiducials->gray_image = CV_Image__create(image_size, CV__depth_8u, 1);
    fiducials->green = CV_Scalar__rgb(0.0, 255.0, 0.0);
    fiducials->mappings = &mappings[0];
    fiducials->origin = CV_Point__create(0, 0);
    fiducials->original_image = original_image;
    fiducials->purple = CV_Scalar__rgb(255.0, 0.0, 255.0);
    fiducials->red = CV_Scalar__rgb(255.0, 0.0, 0.0);
    fiducials->references = CV_Point2D32F_Vector__create(8);
    fiducials->sample_points = CV_Point2D32F_Vector__create(64);
    fiducials->size_5x5 = CV_Size__create(5, 5);
    fiducials->size_m1xm1 = CV_Size__create(-1, -1);
    fiducials->storage = storage;
    fiducials->term_criteria = 
      CV_Term_Criteria__create(term_criteria_type, 5, 0.2);

    return fiducials;
}

Unsigned Fiducials__process(Fiducials fiducials) {
    // Clear *storage*:
    CV_Memory_Storage storage = fiducials->storage;
    CV_Memory_Storage__clear(storage);

    // Grab some values from *fiducials*:
    CV_Image debug_image = fiducials->debug_image;
    Unsigned debug_index = fiducials->debug_index;
    CV_Image edge_image = fiducials->edge_image;
    CV_Image gray_image = fiducials->gray_image;
    CV_Image original_image = fiducials->original_image;

    // For *debug_level* 0, we show the original image in color:
    if (debug_index == 0) {
	CV_Image__copy(original_image, debug_image, (CV_Image)0);
    }

    // Convert from color to gray scale:
    Integer channels = CV_Image__channels_get(original_image);

    // Deal with *debug_index* 0:
    if (debug_index == 0) {
	if (channels == 3) {
	    // Original image is color, so a simple copy will work:
	    CV_Image__copy(original_image, debug_image, (CV_Image)0);
	} else if (channels == 1) {
	    // Original image is gray, so we have to convert back to "color":
	    CV_Image__convert_color(original_image,
	      debug_image, CV__gray_to_rgb);
	}
    }

    // Convert *original_image* to gray scale:
    if (channels == 3) {
        // Original image is color, so we need to convert to gray scale:
	CV_Image__convert_color(original_image, gray_image, CV__rgb_to_gray);
    } else if (channels == 1) {
	// Original image is gray, so a simple copy will work:
	CV_Image__copy(original_image, gray_image, (CV_Image)0);
    } else {
        assert(0);
    }

    // Show results of gray scale converion for *debug_index* 1:
    if (debug_index == 1) {
        CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }
    
    // Perform Gaussian blur if requested:
    if (fiducials->blur) {
	CV_Image__smooth(gray_image, gray_image, CV__gaussian, 3, 0, 0.0, 0.0);
    }

    // Show results of Gaussian blur for *debug_index* 2:
    if (debug_index == 2) {
        CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }

    // Perform adpative threshold:
    CV_Image__adaptive_threshold(gray_image, edge_image, 255.0,
      CV__adaptive_thresh_gaussian_c, CV__thresh_binary, 45, 5.0);

    // Show results of adaptive threshold for *debug_index* 3:
    if (debug_index == 3) {
        CV_Image__convert_color(edge_image, debug_image, CV__gray_to_rgb);
    }

    // Find the *edge_image* *contours*:
    CV_Point origin = fiducials->origin;
    Integer header_size = 128;
    CV_Sequence contours = CV_Image__find_contours(edge_image, storage,
      header_size, CV__retr_list, CV__chain_approx_simple, origin);
    if (contours == (CV_Sequence)0) {
	File__format(stderr, "no contours found\n");
    }

    // For *debug_index* 4, show the *edge_image* *contours*:
    if (debug_index == 4) {
	//File__format(stderr, "Draw red contours\n");
	CV_Scalar red = fiducials->red;
	CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
	CV_Image__draw_contours(debug_image,
	  contours, red, red, 2, 2, 8, origin);
    }

    // For the remaining debug steps, we use the original *gray_image*:
    if (debug_index >= 5) {
	CV_Image__convert_color(gray_image, debug_image, CV__gray_to_rgb);
    }

    // Iterate over all of the countours:
    //big :@= map.big
    Unsigned contours_count = 0;
    for (CV_Sequence contour = contours; contour != (CV_Sequence)0;
      contour = CV_Sequence__next_get(contour)) {
	// Keep a count of total countours:
        contours_count += 1;
	//File__format(stderr, "contours_count=%d\n", contours_count);

	static CvSlice whole_sequence;
	CV_Slice CV__whole_seq = &whole_sequence;
	whole_sequence = CV_WHOLE_SEQ;

	// Perform a polygon approximation of {contour}:
	Integer arc_length =
	  (Integer)(CV_Sequence__arc_length(contour, CV__whole_seq, 1) * 0.02);
	CV_Sequence polygon_contour =
	  CV_Sequence__approximate_polygon(contour,
	  header_size, storage, CV__poly_approx_dp, arc_length, 0.0);
	if (debug_index == 5) {
	    //File__format(stderr, "Draw green contours\n");
	    CV_Scalar green = fiducials->green;
	    CV_Image__draw_contours(debug_image,
	      polygon_contour, green, green, 2, 2, 1, origin);
	}

	// If we have a 4-sided polygon with an area greater than 500 square
	// pixels, we can explore to see if we have a tag:
	if (CV_Sequence__total_get(polygon_contour) == 4 &&
	  fabs(CV_Sequence__contour_area(polygon_contour,
	  CV__whole_seq, 0)) > 500.0 &&
	  CV_Sequence__check_contour_convexity(polygon_contour)) {
	    // For debugging, display the polygons in red:
	    //File__format(stderr, "Have 4 sides > 500i\n");

	    // Just show the fiducial outlines for *debug_index* of 6:
	    if (debug_index == 6) {
		CV_Scalar red = fiducials->red;
		CV_Image__draw_contours(debug_image,
		  polygon_contour, red, red, 2, 2, 1, origin);
	    }

	    // Copy the 4 corners from {poly_contour} to {corners_vector}:
	    CV_Point2D32F_Vector corners_vector = fiducials->corners_vector;
	    for (Unsigned index = 0; index < 4; index++) {
		CV_Point2D32F corner =
		  CV_Point2D32F_Vector__fetch1(corners_vector, index);
		CV_Point point =
		  CV_Sequence__point_fetch1(polygon_contour, index);
		CV_Point2D32F__point_set(corner, point);

		if (debug_index == 6) {
		    //File__format(stderr,
		    //  "point[%d] x:%f y:%f\n", index, point->x, point->y);
		}
	    }

	    // Now find the sub pixel corners of {corners_vector}:
	    CV_Image__find_corner_sub_pix(gray_image, corners_vector, 4,
	      fiducials->size_5x5, fiducials->size_m1xm1,
	      fiducials->term_criteria);

	    // Ensure that the corners are in a counter_clockwise direction:
	    CV_Point2D32F_Vector__corners_normalize(corners_vector);

	    // For debugging show the 4 corners of the possible tag where
	    //corner0=red, corner1=green, corner2=blue, corner3=purple:
	    if (debug_index == 7) {
		for (Unsigned index = 0; index < 4; index++) {
		    CV_Point point =
		      CV_Sequence__point_fetch1(polygon_contour, index);
		    Integer x = CV_Point__x_get(point);
		    Integer y = CV_Point__y_get(point);
		    CV_Scalar color = (CV_Scalar)0;
		    String text = (String)0;
		    switch (index) {
		      case 0:
			color = fiducials->red;
			text = "red";
			break;
		      case 1:
			color = fiducials->green;
			text = "green";
			break;
		      case 2:
			color = fiducials->blue;
			text = "blue";
			break;
		      case 3:
			color = fiducials->purple;
			text = "purple";
			break;
		      default:
			assert(0);
		    }
		    CV_Image__cross_draw(debug_image, x, y, color);
		    File__format(stderr,
		      "poly_point[%d]=(%d:%d) %s\n", index, x, y, text);
		}
	    }

	    // Compute the 8 reference points for deciding whether the
	    // polygon is "tag like" in its borders:
	    CV_Point2D32F_Vector references =
	      Fiducials__references_compute(fiducials, corners_vector);

	    // Now sample the periphery of the tag and looking for the
	    // darkest white value (i.e. minimum) and the lightest black
	    // value (i.e. maximum):
	    Integer white_darkest =
	      CV_Image__points_minimum(gray_image, references, 0, 3);
	    Integer black_lightest =
	      CV_Image__points_maximum(gray_image, references, 4, 7);

	    // {threshold} should be smack between the two:
	    Integer threshold = (white_darkest + black_lightest) / 2;
	    
	    // For debugging, show the 8 points that are sampled around the
	    // the tag periphery to even decide whether to do further testing.
	    // Show "black" as green crosses, and "white" as green crosses:
	    if (debug_index == 8) {
		CV_Scalar red = fiducials->red;
		CV_Scalar green = fiducials->green;
		for (Unsigned index = 0; index < 8; index++) {
		    CV_Point2D32F reference =
		      CV_Point2D32F_Vector__fetch1(references, index);
		    Integer x = CV__round(CV_Point2D32F__x_get(reference));
		    Integer y = CV__round(CV_Point2D32F__y_get(reference));
		    Integer value =
		      CV_Image__point_sample(gray_image, reference);
		    CV_Scalar color = red;
		    if (value < threshold) {
		        color = green;
		    }
		    CV_Image__cross_draw(debug_image, x, y, color);
		    File__format(stderr, "ref[%d:%d]:%d\n", x, y, value);
		}
	    }

	    // If we have enough contrast keep on trying for a tag match:
	    if (black_lightest < white_darkest) {
		// We have a tag to try:

		// Now it is time to read all the bits of the tag out:
		CV_Point2D32F_Vector sample_points = fiducials->sample_points;

		// Now compute the locations to sample for tag bits:
		Fiducials__sample_points_compute(corners_vector, sample_points);

		// Extract all 64 tag bit values:
		Logical *tag_bits = &fiducials->tag_bits[0];
		for (Unsigned index = 0; index < 64; index++) {
		    // Grab the pixel value and convert into a {bit}:
		    CV_Point2D32F sample_point =
		      CV_Point2D32F_Vector__fetch1(sample_points, index);
		    Integer value =
		      CV_Image__point_sample(gray_image, sample_point);
		    Logical bit = (value < threshold);
		    tag_bits[index] = bit;

		    // For debugging:
		    if (debug_index == 9) {
			CV_Scalar red = fiducials->red;
			CV_Scalar green = fiducials->green;
			CV_Scalar cyan = fiducials->cyan;
			CV_Scalar blue = fiducials->blue;

			// Show white bits as {red} and black bits as {green}:
			CV_Scalar color = red;
			if (bit) {
			    color = green;
			}

			//Show where bit 0 and 7 are:
			if (index == 0) {
			    // Bit 0 is {cyan}:
			    color = cyan;
			}
			if (index == 7) {
			    // Bit 7 is {blue}:
			    color = blue;
			}

			// Now splat a cross of {color} at ({x},{y}):
			Integer x =
			  CV__round(CV_Point2D32F__x_get(sample_point));
			Integer y =
			  CV__round(CV_Point2D32F__y_get(sample_point));
			CV_Image__cross_draw(debug_image, x, y, color);
		    }
		}

		//tag_bits :@= extractor.tag_bits
		//bit_field :@= extractor.bit_field
		//tag_bytes :@= extractor.tag_bytes

		// Now we iterate through the 4 different mapping
		// orientations to see if any one of the 4 mappings match:
		Logical **mappings = fiducials->mappings;
		Unsigned mappings_size = 4;
		for (Unsigned direction_index = 0;
		  direction_index < mappings_size; direction_index++) {
		    // Grab the mapping:
		    Logical *mapping = mappings[direction_index];
		    //File__format(stderr,
		    //  "mappings[%d]:0x%x\n", direction_index, mapping);


		    Logical mapped_bits[64];
		    for (Unsigned i = 0; i < 64; i++) {
			 mapped_bits[mapping[i]] = tag_bits[i];
		    }

		    // Fill in tag bytes;
		    Unsigned tag_bytes[8];
		    for (Unsigned i = 0; i < 8; i++) {
			Unsigned byte = 0;
			for (Unsigned j = 0; j < 8; j++) {
			    if (mapped_bits[(i<<3) + j]) {
				byte |= 1 << j;
			    }
			}
			tag_bytes[i] = byte;
		    }
		    if (debug_index == 10) {
			File__format(stderr, "dir=%d Tag[0]=%d Tag[1]=%d\n",
			  direction_index, tag_bytes[0], tag_bytes[1]);
		    }

		    // Now we need to do some FEC (Forward Error Correction):
		    FEC fec = fiducials->fec;
		    if (FEC__correct(fec, tag_bytes, 8)) {
			// We passed FEC:
			if (debug_index == 10) {
			    File__format(stderr, "FEC correct\n");
			}

			// Now see if the two CRC's match:
			Unsigned computed_crc = CRC__compute(tag_bytes, 2);
			Unsigned tag_crc = (tag_bytes[3] << 8) | tag_bytes[2];
			if (computed_crc = tag_crc) {
			    // Yippee!!! We have a tag:
			    // Compute {tag_id} from the the first two bytes
			    // of {tag_bytes}:
			    Unsigned tag_id =
			      (tag_bytes[1] << 8) | tag_bytes[0];

			    if (debug_index == 10) {
				File__format(stderr,
				  "CRC correct, Tag=%d\n", tag_id);
			    }

			    // Lookup {tag} from {tag_id}:
			    // tag :@= null@Tag
			    // if tag_exists@(map, tag_id)
			    //    tag :=
			    //      tag_lookup@(map, tag_id,"Extract@Extractor")
			    // else
				// This actually needs a little discussion.
				// The actual x, y, and angle for this tag
				// is not known until after {map_update@Map}()
				// is called at the end of this routine.
				// We set {tag.x} to {big} to remember that
				// this tag is essentially uninitialized.
				// After the call to {map_update@Map}(), we
				// quickly verify that each {Tag} in {tags}
				// has had its x, y, and angle initialized.
				//tag := tag_create@(map,
				//  tag_id, 0.0, 0.0, 0.0, 7.8125, "Extractor")
				//tag.xx := big

			    // Load {direction_index} and {corners_vector}
			    // into {tag}:
			    //call record@(tag,
			    //  direction_index, corners_vector, indent1)
			    //call append@(tags, tag)

			    if (debug_index == 10) {
			        //call d@(form@("%t%\n\") / f@(tag))
				//Integer xx :@= CV__round(tag.center_x)
				//yy :@= round@CV(tag.center_y)
				///ncyan :@= extractor.cyan
				//call cross_draw@(debug_image, xx, yy, cyan)
			    }
			}
		    }
		}
	    }
	}
    }

    return 0;
}

void CV_Point2D32F_Vector__corners_normalize(CV_Point2D32F_Vector corners) {
    // This routine will ensure that {corners} are ordered
    // in the counter-clockwise direction.

    if (CV_Point2D32F_Vector__is_clockwise(corners)) {
	// Extract two corners to be swapped:
	CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
	CV_Point2D32F corner3 = CV_Point2D32F_Vector__fetch1(corners, 3);

	// Extract X and Y for both corners:
	Double x1 = CV_Point2D32F__x_get(corner1);
	Double y1 = CV_Point2D32F__y_get(corner1);
	Double x3 = CV_Point2D32F__x_get(corner3);
	Double y3 = CV_Point2D32F__y_get(corner3);

	// Swap contents of {corner1} and {corner3}:
	CV_Point2D32F__x_set(corner1, x3);
	CV_Point2D32F__y_set(corner1, y3);
	CV_Point2D32F__x_set(corner3, x1);
	CV_Point2D32F__y_set(corner3, y1);
    }
}

Logical CV_Point2D32F_Vector__is_clockwise(CV_Point2D32F_Vector corners) {

    // Extract the four corners:
    CV_Point2D32F corner0 = CV_Point2D32F_Vector__fetch1(corners, 0);
    CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
    CV_Point2D32F corner2 = CV_Point2D32F_Vector__fetch1(corners, 2);

    // Extract X and Y for all four corners:
    Double x0 = CV_Point2D32F__x_get(corner0);
    Double y0 = CV_Point2D32F__y_get(corner0);
    Double x1 = CV_Point2D32F__x_get(corner1);
    Double y1 = CV_Point2D32F__y_get(corner1);
    Double x2 = CV_Point2D32F__x_get(corner2);
    Double y2 = CV_Point2D32F__y_get(corner2);

    // Create two vectors from the first two lines of the polygon:
    Double v1x = x1 - x0;
    Double v1y = y1 - y0;
    Double v2x = x2 - x1;
    Double v2y = y2 - y1;

    // Determine the sign of the Z coordinate of the cross product:
    Double z = v1x * v2y - v2x * v1y;

    // If the Z coordinate is negative, to reverse the sequence of the corners:
    return z < 0.0;
 }

CV_Point2D32F_Vector Fiducials__references_compute(
  Fiducials fiducials, CV_Point2D32F_Vector corners) {

    // This routine will use the 4 corner points in {corners} to
    // compute 8 reference points that returned.  The first 4 reference
    // points will be just outside of the quadrateral formed by {corners}
    // (i.e. the white bounding box) and the last 4 reference points are
    // on the inside (i.e. the black bounding box).

    // Extract the 8 references from {references}:
    CV_Point2D32F_Vector references = fiducials->references;
    CV_Point2D32F reference0 = CV_Point2D32F_Vector__fetch1(references, 0);
    CV_Point2D32F reference1 = CV_Point2D32F_Vector__fetch1(references, 1);
    CV_Point2D32F reference2 = CV_Point2D32F_Vector__fetch1(references, 2);
    CV_Point2D32F reference3 = CV_Point2D32F_Vector__fetch1(references, 3);
    CV_Point2D32F reference4 = CV_Point2D32F_Vector__fetch1(references, 4);
    CV_Point2D32F reference5 = CV_Point2D32F_Vector__fetch1(references, 5);
    CV_Point2D32F reference6 = CV_Point2D32F_Vector__fetch1(references, 6);
    CV_Point2D32F reference7 = CV_Point2D32F_Vector__fetch1(references, 7);

    // Extract the 4 corners from {corners}:
    CV_Point2D32F corner0 = CV_Point2D32F_Vector__fetch1(corners, 0);
    CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
    CV_Point2D32F corner2 = CV_Point2D32F_Vector__fetch1(corners, 2);
    CV_Point2D32F corner3 = CV_Point2D32F_Vector__fetch1(corners, 3);

    // Extract the x and y references from {corner0} through {corner3}:
    Double x0 = CV_Point2D32F__x_get(corner0);
    Double y0 = CV_Point2D32F__y_get(corner0);
    Double x1 = CV_Point2D32F__x_get(corner1);
    Double y1 = CV_Point2D32F__y_get(corner1);
    Double x2 = CV_Point2D32F__x_get(corner2);
    Double y2 = CV_Point2D32F__y_get(corner2);
    Double x3 = CV_Point2D32F__x_get(corner3);
    Double y3 = CV_Point2D32F__y_get(corner3);

    Double dx21 = x2 - x1;
    Double dy21 = y2 - y1;
    Double dx30 = x3 - x0;
    Double dy30 = y3 - y0;

    // Determine the points ({xx0, yy0}) and ({xx1, yy1}) that determine
    // a line parrallel to one side of the quadralatal:
    Double xx0 = x1 + dx21 * 5.0 / 20.0;
    Double yy0 = y1 + dy21 * 5.0 / 20.0;
    Double xx1 = x0 + dx30 * 5.0 / 20.0;
    Double yy1 = y0 + dy30 * 5.0 / 20.0;

    // Set the outside and inside reference points along the line
    // through points ({xx0, yy0}) and ({xx1, yy1}):
    Double dxx10 = xx1 - xx0;
    Double dyy10 = yy1 - yy0;
    CV_Point2D32F__x_set(reference0, xx0 + dxx10 * -1.0 / 20.0);
    CV_Point2D32F__y_set(reference0, yy0 + dyy10 * -1.0 / 20.0);
    CV_Point2D32F__x_set(reference4, xx0 + dxx10 * 1.0 / 20.0);
    CV_Point2D32F__y_set(reference4, yy0 + dyy10 * 1.0 / 20.0);
    CV_Point2D32F__x_set(reference1, xx0 + dxx10 * 21.0 / 20.0);
    CV_Point2D32F__y_set(reference1, yy0 + dyy10 * 21.0 / 20.0);
    CV_Point2D32F__x_set(reference5, xx0 + dxx10 * 19.0 / 20.0);
    CV_Point2D32F__y_set(reference5, yy0 + dyy10 * 19.0 / 20.0);

    // Determine the points ({xx2, yy2}) and ({xx3, yy3}) that determine
    //a line parrallel to the other side of the quadralatal:
    Double xx2 = x1 + dx21 * 15.0 / 20.0;
    Double yy2 = y1 + dy21 * 15.0 / 20.0;
    Double xx3 = x0 + dx30 * 15.0 / 20.0;
    Double yy3 = y0 + dy30 * 15.0 / 20.0;

    // Set the outside and inside reference points along the line
    // through points ({xx2, yy2}) and ({xx3, yy3}):
    Double dxx32 = xx3 - xx2;
    Double dyy32 = yy3 - yy2;
    CV_Point2D32F__x_set(reference2, xx2 + dxx32 * -1.0 / 20.0);
    CV_Point2D32F__y_set(reference2, yy2 + dyy32 * -1.0 / 20.0);
    CV_Point2D32F__x_set(reference6, xx2 + dxx32 * 1.0 / 20.0);
    CV_Point2D32F__y_set(reference6, yy2 + dyy32 * 1.0 / 20.0);
    CV_Point2D32F__x_set(reference3, xx2 + dxx32 * 21.0 / 20.0);
    CV_Point2D32F__y_set(reference3, yy2 + dyy32 * 21.0 / 20.0);
    CV_Point2D32F__x_set(reference7, xx2 + dxx32 * 19.0 / 20.0);
    CV_Point2D32F__y_set(reference7, yy2 + dyy32 * 19.0 / 20.0);

    return references;
}

Integer CV_Image__points_maximum(CV_Image image,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index) {

    // This routine will sweep from {start_index} to {end_index} through
    // {points}.  Using each selected point in {points}, the corresponding
    // value in {image} is sampled.  The minimum of the sampled point is
    // returned.

    // Start with a big value move it down:
    Integer result = 0;

    // Iterate across the {points} from {start_index} to {end_index}:
    for (Unsigned index = start_index; index <= end_index; index++) {
	CV_Point2D32F point = CV_Point2D32F_Vector__fetch1(points, index);
	Integer value = CV_Image__point_sample(image, point);
	//call d@(form@("max[%f%:%f%]:%d%\n\") %
	//  f@(point.x) % f@(point.y) / f@(value))
	if (value > result) {
	// New maximum value:
	    result = value;
	}
    }
    return result;
}


Integer CV_Image__points_minimum(CV_Image image,
  CV_Point2D32F_Vector points, Unsigned start_index, Unsigned end_index) {

    // This routine will sweep from {start_index} to {end_index} through
    // {points}.  Using each selected point in {points}, the corresponding
    // value in {image} is sampled.  The minimum of the sampled point is
    // returned.

    // Start with a big value move it down:
    Integer result = 0x7fffffff;

    // Iterate across the {points} from {start_index} to {end_index}:
    for (Unsigned index = start_index; index <= end_index; index++) {
	CV_Point2D32F point = CV_Point2D32F_Vector__fetch1(points, index);
	Integer value = CV_Image__point_sample(image, point);
	if (value < result) {
	    // New minimum value:
	    result = value;
	}
    }
    return result;
}

Integer CV_Image__point_sample(CV_Image image, CV_Point2D32F point) {
    // This routine will return a sample ...

    Integer x = CV__round(CV_Point2D32F__x_get(point));
    Integer y = CV__round(CV_Point2D32F__y_get(point));
    Integer center = CV_Image__gray_fetch(image, x, y);
    Integer left = CV_Image__gray_fetch(image, x - 1, y);
    Integer right = CV_Image__gray_fetch(image, x + 1, y);
    Integer lower = CV_Image__gray_fetch(image, x, y - 1);
    Integer upper = CV_Image__gray_fetch(image, x, y + 1);
    Integer result = -1;
    if (center >= 0 && left >= 0 && right >= 0 && lower >= 0 && upper >= 0) {
	result = (center + left + right + lower + upper) / 5;
    }
    return result;
}

void Fiducials__sample_points_compute(
  CV_Point2D32F_Vector corners, CV_Point2D32F_Vector sample_points) {

    // This routine will use the 4 corners in {corners} as a quadralateral
    // to compute an 8 by 8 grid of tag bit sample points and store the
    // results into the the 64 preallocated {CV_Point2D32F} objects in
    // {sample_points}.  The quadralateral must be convex and in the
    // counter-clockwise direction.  Bit 0 will be closest to corners[1],
    // bit 7 will be closest to corners[0], bit 56 closest to corners[2] and
    // bit 63 closest to corners[3].

    CV_Point2D32F corner0 = CV_Point2D32F_Vector__fetch1(corners, 0);
    CV_Point2D32F corner1 = CV_Point2D32F_Vector__fetch1(corners, 1);
    CV_Point2D32F corner2 = CV_Point2D32F_Vector__fetch1(corners, 2);
    CV_Point2D32F corner3 = CV_Point2D32F_Vector__fetch1(corners, 3);

    // Extract the x and y references from {corner0} through {corner3}:
    Double x0 = CV_Point2D32F__x_get(corner0);
    Double y0 = CV_Point2D32F__y_get(corner0);
    Double x1 = CV_Point2D32F__x_get(corner1);
    Double y1 = CV_Point2D32F__y_get(corner1);
    Double x2 = CV_Point2D32F__x_get(corner2);
    Double y2 = CV_Point2D32F__y_get(corner2);
    Double x3 = CV_Point2D32F__x_get(corner3);
    Double y3 = CV_Point2D32F__y_get(corner3);

    // Figure out the vector directions {corner1} to {corner2}, as well as,
    // the vector from {corner3} to {corner0}.  If {corners} specify a
    // quadralateral, these vectors should be approximately parallel:
    Double dx21 = x2 - x1;
    Double dy21 = y2 - y1;
    Double dx30 = x3 - x0;
    Double dy30 = y3 - y0;

    // {index} will cycle through the 64 sample points in {sample_points}:
    Unsigned index = 0;

    // There are ten rows (or columns) enclosed by the quadralateral.
    // (The outermost "white" rows and columns are not enclosed by the
    // quadralateral.)  Since we want to sample the middle 8 rows (or
    // columns), We want a fraction that goes from 3/20, 5/20, ..., 17/20.
    // The fractions 1/20 and 19/20 would correspond to a black border,
    // which we do not care about:
    Double i_fraction = 3.0 / 20.0;
    Double i_increment = 2.0 / 20.0;

    // Loop over the first axis of the grid:
    Unsigned i = 0;
    while (i < 8) {

	// Compute ({xx1},{yy1}) which is a point that is {i_fraction} between
	// ({x1},{y1}) and ({x2},{y2}), as well as, ({xx2},{yy2}) which is a
	// point that is {i_fraction} between ({x0},{y0}) and ({x3},{y3}).
	Double xx1 = x1 + dx21 * i_fraction;
        Double yy1 = y1 + dy21 * i_fraction;
        Double xx2 = x0 + dx30 * i_fraction;
        Double yy2 = y0 + dy30 * i_fraction;

	// Compute the vector from ({xx1},{yy1}) to ({xx2},{yy2}):
	Double dxx21 = xx2 - xx1;
	Double dyy21 = yy2 - yy1;

	// As with {i_fraction}, {j_fraction} needs to sample the
	// the data stripes through the quadralateral with values
	// that range from 3/20 through 17/20:
	Double j_fraction = 3.0 / 20.0;
	Double j_increment = 2.0 / 20.0;

	// Loop over the second axis of the grid:
	Unsigned j = 0;
	while (j < 8) {
	    // Fetch next {sample_point}:
	    CV_Point2D32F sample_point =
	      CV_Point2D32F_Vector__fetch1(sample_points, index);
	    index = index + 1;

            // Write the rvGrid position into the rvGrid array:
	    CV_Point2D32F__x_set(sample_point, xx1 + dxx21 * j_fraction);
	    CV_Point2D32F__y_set(sample_point, yy1 + dyy21 * j_fraction);

	    // Increment {j_faction} to the sample point:
	    j_fraction = j_fraction + j_increment;
	    j = j + 1;
	}

	// Increment {i_fraction} to the next sample striple:
	i_fraction = i_fraction + i_increment;
	i = i + 1;
    }

    CV_Point2D32F sample_point0 =
      CV_Point2D32F_Vector__fetch1(sample_points, 0);
    CV_Point2D32F sample_point7 =
      CV_Point2D32F_Vector__fetch1(sample_points, 7);
    CV_Point2D32F sample_point56 =
      CV_Point2D32F_Vector__fetch1(sample_points, 56);
    CV_Point2D32F sample_point63 =
      CV_Point2D32F_Vector__fetch1(sample_points, 63);

    // clockwise direction.  Bit 0 will be closest to corners[1], bit 7
    // will be closest to corners[0], bit 56 closest to corners[2] and
    // bit 63 closest to corners[3].

    //Fiducials__sample_points_helper("0:7", corner0, sample_point7);
    //Fiducials__sample_points_helper("1:0", corner0, sample_point0);
    //Fiducials__sample_points_helper("2:56", corner0, sample_point56);
    //Fiducials__sample_points_helper("3:63", corner0, sample_point63);
}


void Fiducials__sample_points_helper(
  String label, CV_Point2D32F corner, CV_Point2D32F sample_point) {
    Double corner_x = CV_Point2D32F__x_get(corner);
    Double corner_y = CV_Point2D32F__y_get(corner);
    Double sample_point_x = CV_Point2D32F__x_get(sample_point);
    Double sample_point_y = CV_Point2D32F__y_get(sample_point);
    File__format(stderr, "Label: %s corner: %f:%f sample_point %f:%f\n",
      label, (Integer)corner_x, (Integer)corner_y,
      (Integer)sample_point_x, (Integer)sample_point_y);
}

void Fiducials__tag_record(Unsigned direction, CV_Point2D32F_Vector vector) {
    // This routine will update the contents {Tag} to contain {direction},
    // and {vector}.  {vector} contains four points that form a convex
    // quadralateral in the counter-clockwise direction.  This routine will
    // compute the diagonal and twist values for {tag} as well.

    // Load up the contents of {tag.corners} from {corners_vector} depending
    // upon {direction}:
    Unsigned offset = 0;
    switch (direction) {
      case 0:
	// North mapping:
	//offset := 2
	offset = 0;
	break;
      case 1:
	// East mapping:
	//offset := 1
	offset = 1;
	break;
      case 2:
	// South mapping:
	//offset := 0
	offset = 2;
  	break;
      case 3:
	// West mapping:
	//offset := 3
	offset = 3;
        break;
      default:
	assert (0);
    }

    // Compute {x_center} and {y_center} and fill in {corners}:
    //tag_corners :@= tag.corners
    for (Unsigned point_index = 0; point_index < 4; point_index++) {
	Unsigned corner_index = 0;
	switch (direction) {
	  case 0:
	    corner_index = (3 - point_index + 2) & 3;
	    break;
	  case 1:
	    corner_index = (3 - point_index + 1) & 3;
	    break;
	  case 2:
	    corner_index = (3 - point_index + 0) & 3;
	    break;
	  case 3:
	    corner_index = (3 - point_index + 3) & 3;
	    break;
	  default:
	    assert(0);
	    break;
	}
	//corner :@= vector[corner_index]
	//x :@= corner.x
	//y :@= corner.y
	//tag_corner :@= tag_corners[point_index]
	//tag_corner.x := x
	//tag_corner.y := y
	//point_index := point_index + 1
    }

    // The comment below is out of date:
    //# The Y axis in is image coordinates goes from 0 at the top to
    //# a positive number as it goes towards the bottom.  This is the
    //# opposite direction from from normal cartisian coordinates where
    //# positive Y goes up.  Because my brain can't cope with angles
    //# unless they are in cartisian coordinates, I negate the Y axis
    //# for the purpose of computing the {twist} angles below.  This
    //# flips the direction of the Y axis.

    Double pi = 3.14159265358979323846;

    // Compute {twist}:
    //tag_corner0 = tag_corners[0]
    //tag_corner1 = tag_corners[1]
    //tag_corner2 = tag_corners[2]
    //tag_corner3 = tag_corners[3]

    // Pull out the X and Y coordinates:
    //x0 :@= tag_corner0.x
    //y0 :@= tag_corner0.y
    //x1 :@= tag_corner1.x
    //y1 :@= tag_corner1.y
    //x2 :@= tag_corner2.x
    //y2 :@= tag_corner2.y
    //x3 :@= tag_corner3.x
    //y3 :@= tag_corner3.y

    // Compute the angle of the tag bottom edge to camera X axis:
    //dx01 :@= x0 - x1
    //dy01 :@= y0 - y1
    //twist1 :@= arc_tangent2@(dy01, dx01)
    //dx32 :@= x3 - x2
    //dy32 :@= y3 - y2
    //twist2 :@= arc_tangent2@(dy32, dx32)

    // We want the average angle of {twist1} and {twist2}.  We have
    // be careful about modular arithmetic issues.  Compute the angle
    // change and add in half of that to get the average angle:
    //twist_change :@= angle_between@(twist1, twist2)
    //twist :@= angle_normalize@(twist1 + twist_change / 2.0)
    //tag.twist := twist

    // Compute the X/Y axis deltas for the two diagonals:
    //dx02 :@= x0 - x2
    //dy02 :@= y0 - y2
    //dx13 :@= x1 - x3
    //dy13 :@= y1 - y3

    // Compute the actual diagonals:
    //diagonal02 :@= square_root@(dx02 * dx02 + dy02 * dy02)
    //diagonal13 :@= square_root@(dx13 * dx13 + dy13 * dy13)

    // Compute the average diagonal:
    //diagonal :@= (diagonal02 + diagonal13) / 2.0
    //tag.diagonal := diagonal

    // Compute the center by averagine all for corners:
    //center_x :@= (x0 + x1 + x2 + x3) / 4.0
    //center_y :@= (y0 + y1 + y2 + y2) / 4.0
    //tag.center_x := center_x
    //tag.center_y := center_y

    //if trace
      //call d@(form@(
      //  "%p%id=%d% c0=%2f%,%2f% c1=%2f%,%2f% c2=%2f%,%2f% c3=%2f%,%2f%\n\") %
      //  f@(indent1) % f@(tag.id) %
      //  f@(x0) % f@(y0) % f@(x1) % f@(y1) %
      //  f@(x2) % f@(y2) % f@(x3) / f@(y3))
      //call d@(form@("%p%dx01=%2f% dy01=%2f% dir=%d%\n\") %
      //  f@(indent1) % f@(dx01) % f@(dy01) / f@(tag.direction))
      //call d@(form@("%p%tw1=%2f% tw2=%2f% tw=%2f%\n\") %
      //  f@(indent1) % f@(twist1 * 180.0 / pi) %
      //  f@(twist2 * 180.0 / pi) /  f@(twist * 180.0 / pi))
      //call d@(form@("%p%center_x==%2f% center_y=%2f%\n\") %
      //  f@(indent1) % f@(center_x) / f@(center_y))

    // # For debugging, display everything:
    //if 0f	# 1t
       //#call d@(form@("Mapping:%v% offset:%d%\n\") %
       //#  f@(extractor.mapping_names[direction]) / f@(offset))
       //index :@= 0i
       //while index < 4i
   	    //corner := vector[unsigned@(index)]
	    //vector_x :@= round@CV(corner.x)
	    //vector_y :@= round@CV(corner.y)

	    //#point_x :@= round@CV(get_real_2d@CV(points, index, 0i))
	    //#npoint_y :@= round@CV(get_real_2d@CV(points, index, 1i))
	    //#corner_x :@= round@CV(get_real_2d@CV(corners, index, 0i))
	    //#corner_y :@= round@CV(get_real_2d@CV(corners, index, 1i))

	    //#call d@(form@(
	    //#  "[%d%]: corner_vect=%d%:%d% point=%d%:%d% corner=%d%:%d%\n\") %
	    //#  f@(index) % f@(vector_x) % f@(vector_y) %
	    //#  f@(point_x) % f@(point_y) %
	    //#  f@(corner_x) / f@(corner_y))
	    //index := index + 1i
       //#call d@(form@("corners_vec CW:%l% points CW:%l% corners CW:%l%\n\") %
       //#  f@(is_clockwise@(vector)) % f@(is_clockwise@(points)) /
       //#  f@(is_clockwise@(corners)))

    //if trace
       //call d@(form@("%p%<=record@Tag(T%d%, *)\n\") % f@(indent) / f@(tag.id))
}
		  
// *Neighbor* routines:

Neighbor Neighbor__create(Tag origin, Tag target, Float distance,
  Float target_angle, Float target_twist, Float goodness) {
    Neighbor neighbor = Memory__new(Neighbor);
    neighbor->distance = distance;
    neighbor->goodness = goodness;
    neighbor->target = target;
    neighbor->origin = origin;
    neighbor->target_angle = target_angle;
    neighbor->target_twist = target_twist;
    return neighbor;
}

void Neighbor__write(Neighbor neighbor, File out_file) {
    Float pi = (Float)3.14159265;
    Float radians_to_degrees = 180.0 / pi;

    File__format(out_file, "  <Neighbor");
    File__format(out_file, " Origin=\"%d\"", neighbor->origin);
    File__format(out_file, " Target=\"%d\"", neighbor->target);
    File__format(out_file, " Distance=\"%f\"", neighbor->distance);
    File__format(out_file,
      " Target_Angle=\"%f\"", neighbor->target_angle * radians_to_degrees);
    File__format(out_file,
      " Target_Twist=\"%f\"", neighbor->target_angle * radians_to_degrees);
    File__format(out_file, " Goodness=\"%f\"", neighbor->goodness);
    File__format(out_file, " />\n");
}

// *Tag* routines:

Tag Tag__create(Unsigned id) {
    Tag tag =  Memory__new(Tag);
    tag->floor_angle = (Float)0.0;
    tag->floor_x = (Float)0.0;
    tag->floor_y = (Float)0.0;
    tag->id = id;
    tag->neighbors = List__new();
    return tag;
}

void Tag__Write(Tag tag, File out_file) {
    Float pi = (Float)3.14159265;
    Float radians_to_degrees = 180.0 / pi;

    List neighbors = tag->neighbors;
    Unsigned size = List__size(neighbors);

    File__format(out_file, " <Tag");
    File__format(out_file, " Id=\"%d\"", tag->id);
    File__format(out_file,
      "Floor_Angle=\"%f\"", tag->floor_angle * radians_to_degrees);
    File__format(out_file, " Floor_X=\"%f\"", tag->floor_x);
    File__format(out_file, " Floor_Y=\"%f\"", tag->floor_y);
    File__format(out_file, " Neighbors_Count=\"%d\"", size);
    File__format(out_file, ">\n");

    for (Unsigned index = 0; index < size; index++) {
	Neighbor neighbor = (Neighbor)List__fetch(neighbors, index);
	Neighbor__write(neighbor, out_file);
    }

    File__format(out_file, " </Tag>\n");
}