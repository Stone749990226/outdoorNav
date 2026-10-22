"""Generate an outdoor manifest beside an unchanged indoor deployment."""

import argparse
from pathlib import Path
import yaml


def outdoor_profile(manifest, project):
    result = dict(manifest)
    result['name'] = manifest.get('name', 'lynx') + '-outdoor'
    result['system'] = {k: v for k, v in manifest.get('system', {}).items() if k != 'scene'}
    result['env'] = {k: v for k, v in manifest.get('env', {}).items()
                     if k not in ('SCENE_MAP_ID', 'SCENE_RESTORE_ON_START')}
    result['primitive'] = []
    for item in manifest.get('primitive', []):
        item = dict(item)
        if item['name'] == 'lynx_quadruped':
            item['config'] = {**item.get('config', {}), 'odom_source_topic': '/outdoor/odom'}
        result['primitive'].append(item)
    indoor = {'mapping', 'nav2', 'outdoor_navigation', 'scene_verifier', 'rtabmap', 'icp_odometry'}
    result['service'] = [item for item in manifest.get('service', []) if item['name'] not in indoor]
    # Retained packages must not silently depend on the removed indoor providers.
    def providers(value):
        if isinstance(value, str):
            return {value}
        if isinstance(value, dict):
            return set().union(*(providers(v) for v in value.values())) if value else set()
        if isinstance(value, list):
            return set().union(*(providers(v) for v in value)) if value else set()
        return set()
    for item in [*result['service'], *result['primitive']]:
        config = item.get('config', {})
        references = set().union(*(providers(config.get(key, {})) for key in
                                   ('resolutions', 'provider_ids', 'sensor_providers')))
        if references & indoor:
            raise ValueError('Retained package depends on indoor navigation: ' + item['name'])
    result['service'].append({'name': 'outdoor_navigation',
                              'path': str(project / 'integrations/robonix_outdoor'), 'config': {}})
    # Indoor exploration and skills may depend on the removed map/nav2 services.
    result['skill'] = []
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('indoor_manifest', type=Path)
    parser.add_argument('--project', type=Path, default=Path(__file__).resolve().parents[2])
    args = parser.parse_args()
    source = args.indoor_manifest.resolve()
    output = source.with_name('robonix_manifest.outdoor.yaml')
    if source == output:
        raise ValueError('Use the original indoor manifest as input')
    if output.exists():
        raise FileExistsError('Review or remove the existing outdoor manifest before regenerating: ' + str(output))
    data = outdoor_profile(yaml.safe_load(source.read_text()), args.project.resolve())
    output.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True))
    print(output)


if __name__ == '__main__':
    main()
