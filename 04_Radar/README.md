
# Radar Target Generation and Detection

## Implementation steps for the 2D CFAR process.
First of all, I implemented two outer loops to slide among the rows and columns,
keeping into account the training and guard cells. Then, I added two more loops
to accumulate the noise level in the training cells around the Cell Under Test (CUT).

Obtained these information, the threshold is computed, and, finally, the RDM matrix
is filled checking if the CUT value is higher than the threshold one.



## Selection of Training, Guard cells and offset.
To select the Training, Guard cells and offset, I started from the values proposed
in the project and proceeded tuning them in order to achieve the desired goal.
Finally, I came with the following values:
````
Tr = 10; Td = 8;
Gr = 5; Gd = 5;
offset = 10;
````

## Steps taken to suppress the non-thresholded cells at the edges.
The non-thresholded cells are the ones in the edges of RDM matrix. 
In order to put them to 0, I exploited the following code:

````
RDM(1:Tr+Gr, :) = 0;
RDM(Nr/2-(Tr+Gr):end, :) = 0;
RDM(:, 1:Td+Gd) = 0;
RDM(:, Nd-(Gd+Td):end) = 0;
````

I manually selected the edges, exploiting the knoweldge of the 2D CFAR parameters.



