"""
UCLA NESL Networked Test Bed (NTB) DW1000 Ranging estimation
Author: Paul Martin
Email: pdmartin@ucla.edu

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

# ----- IMPORTS -----
import numpy as np
import scipy
import scipy.stats
import random


# ======== PARTICLE FILTER CLASS =========
class ParticleFilter:

    def __init__(self, num_particles=100, innovate=1.0, limits=[10, 10, 10]):
        self.Np = num_particles
        self.innovation_cov = np.diag([innovate, innovate, innovate])
        self.L_particles = np.zeros(num_particles)
        self.aggregator = {}
        self.xyz_limits = np.array(limits)

        # initialize particle positions & target estimate
        self.xyz_particles = np.multiply(
                [np.random.uniform(0, 1, 3) for x in range(num_particles)],
                self.xyz_limits)
        self.xyz_est = np.array([limits[0]/2.0, limits[1]/2.0, limits[2]/2.0])

        # are we in the middle of an update?
        self.updating = False

    def __del__(self):
        pass

    def addRangeMeasurement(self, uid, xyz, dist, variance):
        # updating mutex
        if self.updating:
            return

        # do we have a measurement from this node in the current period?
        if uid not in self.aggregator:
            # pos, estimate, variance, # samples
            self.aggregator[uid] = [np.array(xyz), dist, variance, 1.0]
            return
        # otherwise add to the aggregator
        self.aggregator[uid][0] += xyz
        self.aggregator[uid][1] += dist
        self.aggregator[uid][2] += variance
        self.aggregator[uid][3] += 1.0

    def update(self):
        # lock aggregator mutex
        self.updating = True

        # if we don't have any new measurements, there's nothing to do here
        if len(self.aggregator) == 0:
            return

        # summarize measurements for each observed uid
        for uid in self.aggregator:
            num_samples = self.aggregator[uid][3]
            self.aggregator[uid][0] = np.divide(self.aggregator[uid][0],
                                                num_samples)
            self.aggregator[uid][1] /= num_samples
            self.aggregator[uid][2] /= num_samples

        for p in range(self.Np):
            # add innovation noise
            self.xyz_particles[p] += self.innovationNoise()

            # if the particle is out of bounds, give it zero likelihood
            xyz = self.xyz_particles[p]
            if any(xyz < [0, 0, 0]) or any(xyz > self.xyz_limits):
                self.L_particles[p] = 0
                continue

            # calculate new likelihood based on measurements since last update
            L = 1.0
            for uid in self.aggregator:
                L *= self.getLikelihood(xyz, self.aggregator[uid])

            # append to likelihood array
            self.L_particles[p] = L

        # In case we got to a bad state where all particles have zero
        # probability, restart PF
        L_sum = np.sum(self.L_particles)

        if L_sum == 0:
            print('Warning: PF in bad state, restarting')
            self.xyz_particles = np.multiply(
                    [np.random.uniform(0, 1, 3) for x in range(self.Np)],
                    self.xyz_limits)
            self.aggregator.clear()
            return

        # normalize likelihoods to sum to 1.
        self.L_particles /= L_sum

        # resample particles based on normalized probability density function
        new_particle_idxs = np.random.choice(range(self.Np), self.Np, True,
                                             p=self.L_particles)
        self.xyz_particles = self.xyz_particles[new_particle_idxs]

        # get new position estimate via particle centroid (mean on axis 0)
        self.xyz_est = np.mean(self.xyz_particles, 0)

        # clear all the measurements we just used
        self.aggregator.clear()

        # done updating
        self.updating = False

        return self.xyz_est

    def getLikelihood(self, xyz, meas):
        # get distance between candidate and measurement point
        meas_origin = meas[0]
        meas_dist = meas[1]
        meas_var = meas[2]
        dist = np.linalg.norm(xyz - meas_origin)
        # print('var = ', meas_var, 'dist = ', dist - meas_dist, 'L = ',
        #      scipy.stats.norm.pdf(dist, meas_dist, meas_var))

        # scipy below takes about 80.3 us to complete
        # return scipy.stats.norm.pdf( dist, meas_dist, meas_var)

        # basic likelihood written below only takes around 3.5 us
        return (1.0 / (meas_var * np.sqrt(2.0 * np.pi))) * np.exp(
            (-(meas_dist - dist) ** 2) / (2.0 * meas_var ** 2))

    def getEstimate(self):
        return self.xyz_est

    def getParticles(self):
        return self.xyz_particles

    def innovationNoise(self):
        return np.random.multivariate_normal([0, 0, 0], self.innovation_cov)


# test particle filter standalone
if __name__ == '__main__':
    estimator = ParticleFilter(100, 2, (10, 10, 10))
    print(estimator.innovationNoise())
