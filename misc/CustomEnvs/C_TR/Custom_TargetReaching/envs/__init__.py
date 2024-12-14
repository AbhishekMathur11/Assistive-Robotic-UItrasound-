from gym.envs.registration import register

register(id='CustomTargetReaching-v0',
    entry_point = 'Custom_TargetReaching.envs:CustomTargetReachingEnv',
    max_episode_steps=200

)