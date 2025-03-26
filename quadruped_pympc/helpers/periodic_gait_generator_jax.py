import os

import jax
import jax.numpy as jnp

import os

dir_path = os.path.dirname(os.path.realpath(__file__))

import sys

sys.path.append(dir_path + '/../')

sys.path.append(dir_path + "/../")

# Parameters for both MPC and simulation
from quadruped_pympc import config


class PeriodicGaitGeneratorJax:
    def __init__(self, duty_factor, step_freq, horizon, mpc_dt):
        self.duty_factor = duty_factor
        self.step_freq = step_freq
        self.horizon = horizon
        self.mpc_dt = mpc_dt

        # Only Trot for now
        self.delta = jnp.array([0.5, 1.0, 1.0, 0.5])

        self.t = jnp.zeros(len(self.delta))
        # self.t = jnp.array([0.0, 0.5, 0.5, 0.0])
        self.n_contact = len(self.delta)

    """def run(self, t, step_freq):
        contact = jnp.zeros(self.n_contact)
        #for leg in range(self.n_contact):
            
        #restart condition
        #t = t.at[0].set(jnp.where(t[0] >= 1.0, 0, t[0]))
        #t = t.at[1].set(jnp.where(t[1] >= 1.0, 0, t[1]))
        #t = t.at[2].set(jnp.where(t[2] >= 1.0, 0, t[2]))
        #t = t.at[3].set(jnp.where(t[3] >= 1.0, 0, t[3]))

        t = t.at[0].set(jnp.where(t[0] >= 1.0, t[0] - 1, t[0]))
        t = t.at[1].set(jnp.where(t[1] >= 1.0, t[1] - 1, t[1]))
        t = t.at[2].set(jnp.where(t[2] >= 1.0, t[2] - 1, t[2]))
        t = t.at[3].set(jnp.where(t[3] >= 1.0, t[3] - 1, t[3]))

        #increase time by dt
        t = t.at[0].set(t[0] + self.mpc_dt*step_freq)
        t = t.at[1].set(t[1] + self.mpc_dt*step_freq)
        t = t.at[2].set(t[2] + self.mpc_dt*step_freq)
        t = t.at[3].set(t[3] + self.mpc_dt*step_freq)

        #contact = contact.at[0].set(jnp.where(t[0] < self.duty_factor, 1.0, 0.0))
        #contact = contact.at[1].set(jnp.where(t[1] < self.duty_factor, 1.0, 0.0))
        #contact = contact.at[2].set(jnp.where(t[2] < self.duty_factor, 1.0, 0.0))
        #contact = contact.at[3].set(jnp.where(t[3] < self.duty_factor, 1.0, 0.0))

        
        contact = contact.at[0].set(jnp.where(t[0] > (1-self.duty_factor), 1.0, 0.0))
        contact = contact.at[1].set(jnp.where(t[1] > (1-self.duty_factor), 1.0, 0.0))
        contact = contact.at[2].set(jnp.where(t[2] > (1-self.duty_factor), 1.0, 0.0))
        contact = contact.at[3].set(jnp.where(t[3] > (1-self.duty_factor), 1.0, 0.0))

        return contact, t"""

    def run(self, t, step_freq):
        contact = jnp.zeros(self.n_contact)
        # for leg in range(self.n_contact):

        # restart condition
        t = t.at[0].set(jnp.where(t[0] >= 1.0, 0, t[0]))
        t = t.at[1].set(jnp.where(t[1] >= 1.0, 0, t[1]))
        t = t.at[2].set(jnp.where(t[2] >= 1.0, 0, t[2]))
        t = t.at[3].set(jnp.where(t[3] >= 1.0, 0, t[3]))

        # increase time by dt
        t = t.at[0].set(t[0] + self.mpc_dt * step_freq)
        t = t.at[1].set(t[1] + self.mpc_dt * step_freq)
        t = t.at[2].set(t[2] + self.mpc_dt * step_freq)
        t = t.at[3].set(t[3] + self.mpc_dt * step_freq)

        contact = contact.at[0].set(jnp.where(t[0] < self.duty_factor, 1.0, 0.0))
        contact = contact.at[1].set(jnp.where(t[1] < self.duty_factor, 1.0, 0.0))
        contact = contact.at[2].set(jnp.where(t[2] < self.duty_factor, 1.0, 0.0))
        contact = contact.at[3].set(jnp.where(t[3] < self.duty_factor, 1.0, 0.0))

        return contact, t

    def set(self, t):
        self.t = t

    def with_newt(self, t):
        self.t = t
        return self

    def get_t(self):
        return self.t

    """def compute_contact_sequence(self, simulation_dt, t, step_freq):
        t_init = jnp.array(t)
        
        contact_sequence = jnp.zeros((self.n_contact, self.horizon))
        new_t = t_init



        #for i in range(self.horizon):
        #    #new_step_freq = jnp.where(i > 2, step_freq, self.step_freq)
        #    #new_contact_sequence, new_t = self.run(new_t, new_step_freq)
        #    new_contact_sequence, new_t = self.run(new_t, step_freq)
        #    contact_sequence = contact_sequence.at[:, i].set(new_contact_sequence)
        
        def body_fn(n, carry):
            new_t, contact_sequence = carry
            new_contact_sequence, new_t = self.run(new_t, step_freq)
            contact_sequence = contact_sequence.at[:, n].set(new_contact_sequence)
            return (new_t, contact_sequence)#, None

        
        init_carry = (new_t, contact_sequence)
        new_t, contact_sequence = jax.lax.fori_loop(1, self.horizon, body_fn, init_carry)
        #new_t, contact_sequence = jax.lax.fori_loop(0, self.horizon, body_fn, init_carry)


        contact = jnp.zeros(self.n_contact)
        contact = contact.at[0].set(jnp.where(t_init[0] > (1-self.duty_factor), 1.0, 0.0))
        contact = contact.at[1].set(jnp.where(t_init[1] > (1-self.duty_factor), 1.0, 0.0))
        contact = contact.at[2].set(jnp.where(t_init[2] > (1-self.duty_factor), 1.0, 0.0))
        contact = contact.at[3].set(jnp.where(t_init[3] > (1-self.duty_factor), 1.0, 0.0))
        contact_sequence = contact_sequence.at[:, 0].set(contact)

        return contact_sequence, new_t"""

    def compute_contact_sequence(self, simulation_dt, t, step_freq):
        t_init = jnp.array(t)

        contact_sequence = jnp.zeros((self.n_contact, self.horizon))
        new_t = t_init

        def body_fn(n, carry):
            new_t, contact_sequence = carry
            new_contact_sequence, new_t = self.run(new_t, step_freq)
            contact_sequence = contact_sequence.at[:, n].set(new_contact_sequence)
            return (new_t, contact_sequence)  # , None

        init_carry = (new_t, contact_sequence)
        new_t, contact_sequence = jax.lax.fori_loop(0, self.horizon, body_fn, init_carry)

        return contact_sequence, new_t


class Gait:
    TROT = 0
    PACE = 1
    BOUNDING = 2
    CIRCULARCRAWL = 3
    BFDIAGONALCRAWL = 4
    BACKDIAGONALCRAWL = 5
    FRONTDIAGONALCRAWL = 6


if __name__ == "__main__":
    # Periodic gait generator

    mpc_dt = config.mpc_params['dt']
    horizon = config.mpc_params['horizon']
    gait = config.simulation_params['gait']
    simulation_dt = config.simulation_params['dt']
    if gait == "trot":
        step_frequency = 2.65
        duty_factor = 0.65
        p_gait = Gait.TROT
    elif gait == "crawl":
        step_frequency = 0.4
        duty_factor = 0.9
        p_gait = Gait.BACKDIAGONALCRAWL
    else:
        step_frequency = 2
        duty_factor = 0.7
        p_gait = Gait.PACE

    pgg = PeriodicGaitGenerator(duty_factor=duty_factor, step_freq=step_frequency, gait_type=p_gait, horizon=horizon)
    pgg_jax = PeriodicGaitGeneratorJax(
        duty_factor=duty_factor, step_freq=step_frequency, horizon=horizon, mpc_dt=mpc_dt
    )
    jitted_pgg = jax.jit(pgg_jax.compute_contact_sequence)
    for j in range(100):
        contact_sequence = pgg.compute_contact_sequence(mpc_dt=mpc_dt, simulation_dt=simulation_dt)
        contact_sequence_jax = pgg_jax.compute_contact_sequence(
            simulation_dt=simulation_dt, t=pgg.get_t(), step_freq=step_frequency
        )
        contact_sequence_jax_jitted = jitted_pgg(simulation_dt, pgg.get_t(), step_frequency)
        print("contact_sequence:\n", contact_sequence)
        print("contact_sequence_jax:\n", contact_sequence_jax[0])
        print("contact_sequence_jax_jitted:\n", contact_sequence_jax_jitted[0])
        pgg.run(simulation_dt)
        print("############")

    breakpoint()
