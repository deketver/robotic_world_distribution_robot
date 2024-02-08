use rand::Rng;
use log::{info};
use log::LevelFilter;
use std::io::Read;
use itertools::Itertools;


/// Structure representing partitioning problem solution via evolutionary algorithm.
pub struct PartitioningProblem{
    /// Vector of weights representing collected items.
    weights: Vec<u32>,
    /// Number of piles, aka markets in our case.
    piles: usize,
    /// Population size.
    pop_size: usize,
    /// Maximum number of generations.
    max_gen: u32,
    /// Crossover probability.
    cx_prob: f32,
    /// Mutation probability.
    mut_prob: f32,
    /// Mutation of a bit in individual.
    mut_flip_prob: f32,
    /// Number of repeats.
    repeats: u32
}

impl PartitioningProblem {
    pub fn new(
        weights: Vec<u32>,
        piles: usize,
        pop_size: usize,
        max_gen: u32,
        cx_prob: f32,
        mut_prob: f32,
        mut_flip_prob: f32,
        repeats: u32
    ) -> Self {
        PartitioningProblem {
            weights,
            piles,
            pop_size,
            max_gen,
            cx_prob,
            mut_prob,
            mut_flip_prob,
            repeats
        }
    }

    pub fn set_weights(&mut self, weights: Vec<u32>) {
        self.weights = weights;
    }

    pub fn set_weights_from_file(&mut self, file_name: &str) {
        let mut weights = Vec::new();
        let mut file = std::fs::File::open(file_name).unwrap();
        let mut contents = String::new();
        file.read_to_string(&mut contents).unwrap();
        for line in contents.lines() {
            weights.push(line.parse::<u32>().unwrap());
        }
        self.weights = weights;
    }

    fn create_individual(&self, ind_len: usize)-> Vec<usize> {
        // individual is represented as a vector of indexes
        let mut rng = rand::thread_rng();
        let mut individual =  Vec::with_capacity(ind_len);
        for _ in 0..ind_len {
            individual.push(rng.gen_range(0..self.piles));
        }
        individual
    }

    fn create_population(&self, ind_len: usize) -> Vec<Vec<usize>> {
        let mut population = Vec::with_capacity(self.pop_size);
        for _ in 0..self.pop_size {
            population.push(self.create_individual(ind_len));
        }
        population
    }

    fn fitness_objective(&self, individual: &Vec<usize>) -> (f32, u32) { // bins: &Vec<usize>
        let bin_weights : Vec<u32> = self.bin_weights(individual);
        let max_bin_weight : u32 = *bin_weights.iter().max().unwrap();
        let min_bin_weight : u32= *bin_weights.iter().min().unwrap();
        let fitness : f32= 1.0/((max_bin_weight as f32 - min_bin_weight as f32).powf(2.0));
        //let fitness : f32= 1.0/((max_bin_weight - min_bin_weight + 1) as f32);
        let objective : u32 = max_bin_weight - min_bin_weight;
        (fitness, objective)
    }

    fn objective(&self, individual: &Vec<usize>) -> u32{
        let bin_weights : Vec<u32> = self.bin_weights(individual);
        let max_bin_weight : u32 = *bin_weights.iter().max().unwrap();
        let min_bin_weight : u32= *bin_weights.iter().min().unwrap();
        max_bin_weight - min_bin_weight
    }

    fn bin_weights(&self, bins_individual: &Vec<usize>)-> Vec<u32> {
        // weights of given bins, aka markets in our case
        //TODO: This could be potentially better initialized
        let mut bin_weights = Vec::with_capacity(self.piles);
        for _ in 0..self.piles {
            bin_weights.push(0);
        }
        for (i, bin) in bins_individual.iter().enumerate() {
            bin_weights[*bin] += self.weights[i];
        }
        bin_weights
    }

    fn tournament_selection(&self, population: &Vec<Vec<usize>>, fitness: &Vec<f32>) -> Vec<Vec<usize>> {
        let mut rng = &mut rand::thread_rng();
        let mut selected: Vec<Vec<usize>> = Vec::with_capacity(self.pop_size);
        for _ in 0..self.pop_size{
            let v: Vec<_>= rand::seq::index::sample(&mut rng, population.len(), 2).into_vec();
            if fitness[v[0]] > fitness[v[1]] {
                selected.push(population[v[0]].clone());
            }
            else {
                selected.push(population[v[1]].clone());
            }
        }
        selected
    }

    fn one_point_crossover(&self, parent1: &Vec<usize>, parent2: &Vec<usize>) -> (Vec<usize>, Vec<usize>) {
        let rng = &mut rand::thread_rng();
        let crossover_point = rng.gen_range(0..parent1.len());
        let mut child1 = parent1.clone();
        let mut child2 = parent2.clone();
        for i in crossover_point..parent1.len() {
            child1[i] = parent2[i];
            child2[i] = parent1[i];
        }
        (child1, child2)
    }

    fn flip_mutate(&self, individual: &mut Vec<usize>){
        let rng = &mut rand::thread_rng();
        for value in individual.iter_mut() {
            if rng.gen::<f32>() < self.mut_flip_prob {
                *value = rng.gen_range(0..self.piles);
            }
        }
    }

    fn crossover(&self, population: &mut Vec<Vec<usize>>) -> Vec<Vec<usize>> {
        let rng = &mut rand::thread_rng();
        let pop1: Vec<_> = population.iter().cloned().step_by(2).collect();
        let pop2: Vec<_> = population.iter().cloned().skip(1).step_by(2).collect();
        let mut offsprings = Vec::new();
        for i in 0..pop1.len() {
            let parent1 = &pop1[i];
            let parent2 = &pop2[i];
            if rng.gen::<f32>() < self.cx_prob {
                let (child1, child2) = self.one_point_crossover(parent1, parent2);
                offsprings.push(child1);
                offsprings.push(child2);
            }
            else {
                offsprings.push(parent1.clone());
                offsprings.push(parent2.clone());
            }
        }
        offsprings
    }

    fn mutate(&self, population: &mut Vec<Vec<usize>>) {
        let rng = &mut rand::thread_rng();
        //let mut new_population = Vec::new();
        for individual in population.iter_mut() {
            if rng.gen::<f32>() < self.mut_prob {
                self.flip_mutate(individual);
            }
        }
    }

    fn mate(&self, population: &mut Vec<Vec<usize>>) -> Vec<Vec<usize>>{
        // crossover and mutation are operators we want to apply on the population
        let mut new_population = self.crossover(population);
        self.mutate(&mut new_population);
        new_population
    }

    pub fn evolutionary_algo_run(&self, population: &mut Vec<Vec<usize>>) -> Vec<Vec<usize>>{
        for generation in 0..self.max_gen{
            let mut fitness: Vec<f32> = Vec::with_capacity(population.len());
            let mut objective: Vec<u32> = Vec::with_capacity(population.len());
            for individual in population.iter() {
                let (fit, obj) = self.fitness_objective(individual); //&self.bin_weights(individual)
                fitness.push(fit);
                objective.push(obj);
            }
            if generation % 100 == 0 {
                info!("Generation: {}, min objective: {:?}", generation, objective.iter().min().unwrap());
            }
            let mut mating_pool = self.tournament_selection(population, &fitness);
            let new_population : Vec<Vec<usize>> = self.mate(&mut mating_pool);
            *population = new_population;
        }
        population.to_vec()
    }

    pub fn main_exec(&self, log_path: &str) -> Vec<usize>{
        let mut best_individuals = Vec::new();
        for run in 0..self.repeats {
            if run == 0{
                let _ = simple_logging::log_to_file(log_path, LevelFilter::Info);
            }
            let mut population : Vec<Vec<usize>>= self.create_population(self.weights.len());
            population = self.evolutionary_algo_run(&mut population);
            let mut fitness: Vec<f32> = Vec::with_capacity(population.len());
            let mut objective: Vec<u32> = Vec::with_capacity(population.len());
            for individual in population.iter() {
                let (fit, obj) = self.fitness_objective(individual);
                fitness.push(fit);
                objective.push(obj);
            }
            let best_individual = population[fitness.iter().position_max_by(|a, b| a.partial_cmp(b).unwrap()).unwrap()].clone();
            best_individuals.push(best_individual.clone());
            info!("Run: {}:", run);
            info!("Objective best individual: {:?}", self.objective(&best_individual));
            info!("Bin Weights: {:?}", self.bin_weights(&best_individual));
        }
        let best_individual = best_individuals[best_individuals.iter().position_min_by(|a, b| self.objective(a).cmp(&self.objective(b))).unwrap() as usize].clone();
        println!("Best individual: {:?}", best_individual);
        info!("Best individual: {:?}", best_individual);
        println!("Best individual objective: {:?}", self.objective(&best_individual));
        return best_individual;
    }
}

pub fn create_eva_problem(){
    let mut problem = PartitioningProblem::new(
        Vec::new(),
        10,
        100,
        1500,
        0.8,
        0.22,
        0.085,
        10
    );
    problem.set_weights_from_file("test_data/partition.txt");
    println!("Weights successfully loaded");
    let _best_solution: Vec<usize> = problem.main_exec("logs/evolutionary_algo_test.log");
}