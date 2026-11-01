#!/usr/bin/env python3
"""Install only the mass-enabled MPC sources after baseline checks and backup."""
from pathlib import Path
import argparse
import datetime
import hashlib
import json
import sys

def digest(data):
    return hashlib.sha256(data).hexdigest()

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("project", type=Path)
    parser.add_argument("--check", action="store_true", help="validate and list changes without writes")
    args = parser.parse_args()
    project = args.project.resolve()
    package = Path(__file__).resolve().parent
    manifest = json.loads((package / "install_manifest.json").read_text())
    if not (project / ".project").is_file():
        sys.exit("STM32プロジェクトが見つかりません。変更なし。")
    changes = []
    for item in manifest:
        source = package / item["source"]
        target = project / item["destination"]
        new = source.read_bytes()
        if digest(new) != item["updated_sha256"]:
            sys.exit("パッケージ内のファイルが一致しません: " + item["source"])
        if target.is_symlink() or project not in target.resolve().parents:
            sys.exit("シンボリックリンクまたはプロジェクト外のパスです。変更なし: " + str(target))
        original = target.read_bytes() if target.is_file() else None
        if original == new:
            continue
        if original is None:
            if item["baseline_sha256"] is not None:
                sys.exit("既存のMPCファイルが見つかりません。変更なし: " + item["destination"])
        elif digest(original) != item["baseline_sha256"]:
            sys.exit("既存ファイルが確認済み版と異なります。個別変更を上書きせず停止しました: "
                     + item["destination"] + "\n現在のファイルに差分を反映する必要があります。")
        changes.append((target, original, new, item["destination"]))
    if not changes:
        print("導入済みです。変更なし。")
        return
    for _, _, _, relative in changes:
        print("追加/更新:", relative)
    if args.check:
        print("確認のみ。ファイルは変更していません。")
        return
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup = project / ".mpc-backups" / ("runtime_mass_" + stamp)
    if project not in backup.resolve().parents:
        sys.exit("バックアップ先がプロジェクト外のため停止しました。変更なし。")
    backup.mkdir(parents=True, exist_ok=False)
    for _, original, _, relative in changes:
        if original is not None:
            saved = backup / relative
            saved.parent.mkdir(parents=True, exist_ok=True)
            saved.write_bytes(original)
    written = []
    try:
        for target, original, new, _ in changes:
            current = target.read_bytes() if target.is_file() else None
            if current != original:
                raise RuntimeError("作業中にファイルが変更されました: " + str(target))
        for target, _, new, _ in changes:
            item = next(entry for entry in changes if entry[0] == target)
            current = target.read_bytes() if target.is_file() else None
            if current != item[1]:
                raise RuntimeError("書込み直前にファイルが変更されました: " + str(target))
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(new)
            written.append(item)
            if target.read_bytes() != new:
                raise RuntimeError("書込み後の一致確認に失敗: " + str(target))
    except Exception:
        for target, original, new, _ in written:
            if target.read_bytes() != new:
                print("外部変更があるため自動復元しません。バックアップを確認してください:", target)
                continue
            if original is None:
                if target.is_file():
                    target.unlink()
            else:
                target.write_bytes(original)
        raise
    print("MPCの質量更新版を導入しました。バックアップ:", backup)
    print("main・タイマー・クロック・.ioc・.cproject・リンカ・.aは変更していません。")
    print("main_integration.txtに従い、質量更新→状態更新→solveをmainへ接続してください。")

if __name__ == "__main__":
    main()
