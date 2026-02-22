Path Planning

Project Description

This project uses the Particle Swarm Optimization (PSO) and Firefly algorithm to find the shortest path between two destinations with obstacles in between. Using PSO, the algorithm runs for 1000 iterations to optimize the path; then, the same problem is solved using the Firefly algorithm, and the results of the two methods are compared.

Installation
	1.	Clone this repository:
git clone <repository_link>
	2.	Install required libraries:
For example, if using Python, you need to install numpy, matplotlib, and scipy.
pip install -r requirements.txt
	3.	Run the code:
python main.py

Usage

When you run the project, you will be asked to input the start and end points. Obstacles are defined on a grid, and the algorithm will compute the shortest path through them after the given number of iterations.

Results

In this project, we compared the performance of the PSO and Firefly algorithms. PSO provided stable results over 1000 iterations, while the Firefly algorithm adapted faster to the obstacle layout. Detailed result graphs and data are provided in the report.

License

This project is licensed under the MIT License.

Contact

If you have any questions, you can reach me at [alturkroshat@gmail.com].
