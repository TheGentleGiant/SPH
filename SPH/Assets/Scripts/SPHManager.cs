using System;
using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Networking;
using Random = UnityEngine.Random;
using Vector3 = UnityEngine.Vector3;

public class SPHManager : MonoBehaviour
{
    public GameObject Prefab;
    public float Radius;
    public float Mass;
    public float RestDensity;
    public float Viscosity;
    public float Drag;
    
    public bool isWallUp;
    public int particleAmount;
    public int particlesPerRow;
    public List<GameObject> walls = new List<GameObject>();

    private float _smoothigRadius = 1.0f;
    private Vector3 _gravity = new Vector3(0.0f, -9.81f, 0.0f);
    private float _gravityMultiplier = 2000f;
    private float _gas = 2000f;
    private float _deltaTime = 0.0008f;
    private float _damping = -0.5f;

    private Particle[] _particles;
    private ParticleCollider[] _colliders;
    private bool isClearing;

    private void Start()
    {
        Initialize();
    }

    private void Update()
    {
        CalculateForce();
        CalculateCollisions();
        ParticleMovement();
    }

    private void Initialize()
    {
        _particles = new Particle[particleAmount];
        _colliders = new ParticleCollider[particleAmount];
        for (int i = 0; i < particleAmount; i++)
        {
            float x = (i % particlesPerRow) + Random.Range(-0.1f, 0.1f);
            float y = (2 * Radius) + (float) ((i / particlesPerRow) / particlesPerRow) * 1.1f;
            float z = ((i / particlesPerRow) % particlesPerRow) + Random.Range(-0.1f, 0.1f);

            GameObject current = Instantiate(Prefab);
            Particle currentParticle = current.AddComponent<Particle>();
            ParticleCollider currentCollider = current.AddComponent<ParticleCollider>();
            _particles[i] = currentParticle;
            _colliders[i] = currentCollider;
            
            current.transform.localScale = Vector3.one * Radius;
            current.transform.position = new Vector3(x,y,z);

            currentParticle.Go = current;
            currentParticle.Position = current.transform.position;
            currentCollider.Position = current.transform.position;
            currentCollider.Right = current.transform.right;
            currentCollider.Up = current.transform.up;
            currentCollider.Scale = current.transform.localScale;

        }
    }

    private void CalculateForce()
    {
        for (int i = 0; i < _particles.Length; i++)
        {
            if (isClearing) {return;}

            for (int j = i; j < _particles.Length;j++)
            {
                Vector3 direction = _particles[j].Position - _particles[i].Position;
                float distance = direction.magnitude;

                _particles[i].Density = ParticleDensity(_particles[i], distance);
                _particles[i].Pressure = _gas * (_particles[i].Density - RestDensity);
            }
        }
    }

    private float ParticleDensity(Particle particle, float distance)
    {
        if (distance < _smoothigRadius)
        {
            return particle.Density += Mass * (315.0f / (64.0f * Mathf.PI * Mathf.Pow(_smoothigRadius, 9.0f))) *
                                       Mathf.Pow(_smoothigRadius - distance, 3.0f);
        }

        return particle.Density;
    }

    private void ParticleMovement()
    {
        for (int i = 0; i < _particles.Length; i++)
        {
            if (isClearing) { return;}
            
            Vector3 forcePressure = Vector3.zero;
            Vector3 forceViscosity = Vector3.zero;

            for (int j = i; j < _particles.Length; j++)
            {
                if (i==j) { continue; }

                Vector3 directioin = _particles[j].Position - _particles[i].Position;
                float distance = directioin.magnitude;

                forcePressure += ParticlePressure(_particles[i], _particles[j], directioin, distance);
                forceViscosity += ParticleViscosity(_particles[i], _particles[j], distance);
            }

            Vector3 forceGravity = _gravity * _particles[i].Density * _gravityMultiplier;

            _particles[i].CombinedForce = forcePressure + forceViscosity + forceGravity;
            _particles[i].Velocity += _deltaTime * (_particles[i].CombinedForce) / _particles[i].Density;
            _particles[i].Position += _deltaTime * (_particles[i].Velocity);
            _particles[i].Go.transform.position = _particles[i].Position;

           
        }
    }

    private Vector3 ParticleViscosity(Particle particle, Particle nextParticle, float distance)
    {
        if (distance < _smoothigRadius)
        {
            return Viscosity * Mass * ((_smoothigRadius - distance)) / nextParticle.Density *
                   (45f / (Mathf.PI * Mathf.Pow(_smoothigRadius, 6.0f))) * (nextParticle.Velocity - particle.Velocity);
        }
        return Vector3.zero;
    }

    private Vector3 ParticlePressure(Particle particle, Particle nextParticle, Vector3 directioin, float distance)
    {
        if (distance < _smoothigRadius)
        {
            return -1 * Mathf.Pow(_smoothigRadius - distance, 2.0f) * Mass * (particle.Pressure + nextParticle.Pressure) /
                   (2.0f * nextParticle.Density) *
                   (-45f / (Mathf.PI * Mathf.Pow(_smoothigRadius, 6.0f))) * (directioin.normalized);
        }
        return Vector3.zero;
    }

    private void CalculateCollisions()
    {
        for (int i = 0; i < _particles.Length; i++)
        {
            for (int j = i; j < _particles.Length; j++)
            {
                if (isClearing || _colliders.Length == 0)
                {
                    return;
                }

                Vector3 penetrationNormal;
                Vector3 penetrationPosition;
                float penetrationLength;
                if (CheckCollision(_colliders[j], _particles[i].Position, Radius, out penetrationNormal, out penetrationPosition, out penetrationLength))
                {
                    _particles[i].Velocity = DampenVelocity(_colliders[j], _particles[i].Velocity, penetrationNormal,
                        1.0f - Drag);
                    _particles[i].Position = penetrationPosition - penetrationNormal * Mathf.Abs(penetrationLength);
                }

                if (CheckCollision(walls[0].gameObject.GetComponent<Collider>(),_particles[i].Position, Radius ))
                {
                    _particles[i].Position.y = walls[0].transform.up.y * (Radius + 0.25f);
                    Debug.Log("Colliding" );
                    _particles[i].Velocity = DampenVelocity(_colliders[j], _particles[i].Velocity, walls[0].transform.up,
                     1.0f - Drag);
                    //_particles[i].Position = penetrationPosition - penetrationNormal * Mathf.Abs(penetrationLength);
                }
            }
        }
    }



    private Vector3 DampenVelocity(ParticleCollider particleCollider, Vector3 velocity, Vector3 penetrationNormal, float drag)
    {
        Vector3 newVelocity = Vector3.Dot(velocity, penetrationNormal) * _damping * penetrationNormal +
                              Vector3.Dot(velocity, particleCollider.Right) *
                              drag * particleCollider.Right + Vector3.Dot(velocity, particleCollider.Up) *
                              Drag * particleCollider.Up;

        return Vector3.Dot(newVelocity, Vector3.forward) * Vector3.forward +
               Vector3.Dot(newVelocity, Vector3.right) * Vector3.right +
               Vector3.Dot(newVelocity, Vector3.up) * Vector3.up;
    }

    private bool CheckCollision(ParticleCollider particleCollider, Vector3 position, float radius, out Vector3 penetrationNormal, out Vector3 penetrationPosition, out float penetrationLength)
    {
        Vector3 colliderProjection = particleCollider.Position - position;

        penetrationNormal = Vector3.Cross(particleCollider.Right, particleCollider.Up);
        penetrationLength = Mathf.Abs(Vector3.Dot(colliderProjection, penetrationNormal)) - (radius / 2.0f);
        penetrationPosition = particleCollider.Position - colliderProjection;

        return penetrationLength < 0.0f
               && Mathf.Abs(Vector3.Dot(colliderProjection, particleCollider.Right)) < particleCollider.Scale.x
               && Mathf.Abs(Vector3.Dot(colliderProjection, particleCollider.Up)) < particleCollider.Scale.y;

    }

    private bool CheckCollision(Collider _meshCollider, Vector3 _position, float radius)
    {
        Transform _colliderTransform = _meshCollider.gameObject.transform;

        return _colliderTransform.position.y > _position.y + Radius/2;
    }
}
